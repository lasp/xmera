.. _filtering-architecture:

Filtering Architecture (filtering_core)
=======================================

.. note::

   This page describes the framework-agnostic Kalman-filter infrastructure
   under ``src/filtering_core`` and the per-filter algorithm libraries under
   ``src/filtering_algorithms``. It explains how the pieces relate, how to use
   a filter, and how to add a new one. The first filter built on this
   infrastructure is :ref:`flybyODuKF`.

Why this exists
---------------

The filtering infrastructure splits that responsibility into three layers:

#. a **framework-agnostic core** (``filtering_core``) — the reusable filter
   machinery, depending only on Eigen and the C++ standard library;
#. a **per-filter algorithm library** (``filtering_algorithms/<filter>``) — the
   concrete dynamics, measurement model, and a plain C++ class that owns the
   filter state;
#. a **thin host adapter** (under ``fswAlgorithms/``) — the only layer that
   knows about xmera messaging and ``SysModel``.

The boundary is enforced by the build: ``filtering_core`` and
``filtering_algorithms/*`` are separate library targets that do **not** link or
include any of ``architecture/messaging``, the message-payload headers,
or ``SysModel``. Code that compiles against those
targets is therefore portable by construction.

Layered structure
-----------------

::

   +-------------------------------------------------------------+
   | xmera host adapter   (fswAlgorithms/.../flybyODuKF.{h,cpp})  |
   |   - SysModel, ReadFunctor<>, Message<>                       |
   |   - marshals MsgPayloads <-> POD I/O types                  |
   +----------------------------+--------------------------------+
                                | owns (pimpl)
                                v
   +-------------------------------------------------------------+
   | algorithm library   (filtering_algorithms/flybyODuKF)       |
   |   - FlybyODuKFSpec        (State, ProcessNoiseCov typedefs)  |
   |   - FlybyODuKFAlgorithm   (stateful, plain C++ class)        |
   |   - *Types.h              (HeadingMeasurement, ... POD)      |
   |   - dynamics closure + HeadingMeasurementModel              |
   +----------------------------+--------------------------------+
                                | built on
                                v
   +-------------------------------------------------------------+
   | filtering_core   (header-only INTERFACE library)            |
   |   - StateVector<Components...> + component tags             |
   |   - concepts: FilterState, Measurement, Dynamics, ...       |
   |   - DynamicsModel / rk4 / propagate                         |
   |   - measurement_queue                                       |
   |   - SRUKF functional core + SrukfInterface<Spec> facade     |
   +-------------------------------------------------------------+
                  depends only on Eigen + the C++ stdlib

filtering_core
~~~~~~~~~~~~~~~

Header-only ``INTERFACE`` library (``src/filtering_core/include/filtering_core``).
Headers are included as ``<filtering_core/...>``.

``state.hpp``
   ``StateVector<Components...>`` is a variadic, type-composed state vector laid
   out contiguously in a single fixed-size ``Eigen::Vector``. Components are
   addressed by their tag type at compile time:

   .. code-block:: cpp

      using FlybyState = filtering::StateVector<filtering::Position<3>,
                                                filtering::Velocity<3>>;  // size 6
      FlybyState s;
      s.set<filtering::Position<3>>(Eigen::Vector3d{1, 2, 3});
      Eigen::Vector3d v = s.get<filtering::Velocity<3>>();

   Component tags (``Position``, ``Velocity``, ``Acceleration``, ``Bias``,
   ``MrpAttitude``, ``AngularRate``, ``Consider``) each carry a compile-time
   ``size``. ``StateVector`` satisfies ``LinearlyCombinable`` via ``add()`` and
   ``scale()`` and exposes ``raw()`` for the underlying Eigen vector.

``concepts.hpp``
   The C++20 concepts that define the plug points, so filters compose by
   *satisfying an interface* rather than by inheritance:

   - ``LinearlyCombinable<T>`` — has ``scale(k)`` and ``add(other)``.
   - ``FilterState<S>`` — ``LinearlyCombinable`` plus ``S::size`` and ``raw()``.
   - ``Measurement<M, State>`` — ``M::size``, ``observation()``,
     ``model(state)``, ``noise()``, ``subtract(a, b)``.
   - ``Dynamics<D, State>`` — invocable ``(double, State) -> State``.

``dynamics_model.hpp``
   ``DynamicsModel<State>`` (a ``std::function`` alias) plus ``rk4()`` and
   ``propagate()`` — concept-constrained RK4 integration usable on any
   ``LinearlyCombinable`` state.

``kalman_filter.hpp``
   The ``SequentialFilter<F, M>`` concept — the contract a Kalman-style filter
   satisfies (``timeUpdate(dt)`` + ``measurementUpdate(m)``).

``measurement_queue.h``
   ``measurement_queue<Measurement, CAPACITY>`` — a bounded, time-ordered
   container (``enqueue`` / ``popEarliest`` / ``clear`` / ``isFull`` /
   ``isEmpty``), plus the free template
   ``apply_sequential(queue, filter, t0, t1)`` (constrained on
   ``SequentialFilter``) that interleaves time and measurement updates over a
   window. Additional scheduling styles (batch, iterated, ...) drop in as new
   free templates without changing the queue.

``srukf_interface.hpp``
   The square-root UKF in two layers:

   - a **functional core** — ``SrukfStorage<State>`` (plain data, all
     fixed-size) and free functions ``srukf::reset<Spec>``,
     ``srukf::predict<Spec, D>``, ``srukf::update<Spec, M>``;
   - a **stateful facade** — ``SrukfInterface<Spec>``, which holds a
     ``SrukfStorage`` and a settable ``dynamics`` member and exposes
     ``reset()`` / ``predict(dt)`` / ``update<M>(m)`` plus setters/getters.

filtering_algorithms/<filter>
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

A ``STATIC`` library per filter, depending only on ``filtering_core``. For
``flybyODuKF`` it contains:

- ``flybyODuKFTypes.h`` — POD input/output structs (``HeadingMeasurement``,
  ``FilterStateOutput``, ``ResidualsOutput``) the host adapter marshals to and
  from. These are the seam the future C shim wraps.
- ``flybyODuKFAlgorithm.h`` — the ``FlybyODuKFAlgorithm`` class: setters and
  getters for tunables, ``reset()``, the ``SequentialFilter`` pair
  (``timeUpdate(dt)`` / ``measurementUpdate(m)``), the queue-driven
  ``enqueueMeasurement`` + ``update(t0, t1)``, and the POD-packaged readouts
  ``getState()`` / ``getLastResiduals()``.
- ``flybyODuKFAlgorithm.cpp`` — the two-body dynamics closure and the
  ``HeadingMeasurementModel`` (which satisfies ``Measurement<M, State>``). These
  internal pieces live in the ``.cpp``, not the header, to keep the public
  surface minimal.

host adapter
~~~~~~~~~~~~

``fswAlgorithms/opticalNavigation/flybyODuKF/flybyODuKF.{h,cpp}`` is the only
xmera-aware layer. It is a ``SysModel`` that owns the message ports and holds
the algorithm behind a ``std::unique_ptr`` (forward-declared, so the
SWIG-parsed header never sees the concept-heavy core). Each ``updateState`` it
reads the input payload into a ``HeadingMeasurement``, enqueues it on the
algorithm (``enqueueMeasurement``), and then drives the filter over the window
with a single ``update(t0, t1)`` call. The algorithm owns the
``measurement_queue`` and runs ``apply_sequential(measurements, filter, t0, t1)``
inside that ``update`` call; ``apply_sequential`` decides per step whether to
``timeUpdate`` (predict) or ``measurementUpdate``. The adapter then writes the
algorithm's ``getState()`` / ``getLastResiduals()`` back into the output
payloads.

Anatomy of a filter: how the pieces compose
--------------------------------------------

Every filter built on ``filtering_core`` is the *same machine with a few holes
filled in*. The core supplies the estimator math, the numerical integrator, the
measurement scheduler, and the C++20 concepts that define the holes; a concrete
filter supplies only what is genuinely filter-specific — what is estimated, how
it evolves, and how it is observed — plus a thin class that wires them together.

Roles, contracts, and concrete fill-ins
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. list-table::
   :header-rows: 1
   :widths: 24 28 24 24

   * - Role
     - Contract (concept)
     - Core consumer
     - FlybyODuKF supplies
   * - What is estimated
     - ``FilterState<S>``
     - ``SrukfInterface<Spec>``, ``propagate``
     - ``StateVector<Position<3>, Velocity<3>>``
   * - How the state evolves
     - ``Dynamics<D, State>``
     - ``rk4`` / ``propagate`` (in ``srukf::predict``)
     - two-body gravity closure
   * - How the state is observed
     - ``Measurement<M, State>``
     - ``srukf::update``
     - ``HeadingMeasurementModel``
   * - Estimator binding
     - ``SrukfSpec`` (``State`` + ``ProcessNoiseCov``)
     - ``SrukfInterface<Spec>``
     - ``FlybyODuKFSpec``
   * - The drivable filter
     - ``SequentialFilter<F, M>`` (``timeUpdate`` + ``measurementUpdate``)
     - ``apply_sequential`` (free template)
     - ``FlybyODuKFAlgorithm``
   * - When to predict vs. update
     - — (concrete, not a concept)
     - ``measurement_queue`` (container) + ``apply_sequential`` (free function)
     - ``FlybyODuKFAlgorithm`` owns one ``measurement_queue<HeadingMeasurement, 1>`` and runs ``apply_sequential(measurements, *this, t0, t1)`` from ``update(t0, t1)``

The first three rows are the *plugs* a new filter must provide; everything in
the "Core consumer" column is reused unchanged.

Static view: concepts as sockets
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Each concept is a socket in the core; a concrete type plugs in by *satisfying*
it — no inheritance, checked at compile time:

::

   generic socket (concept in filtering_core)    concrete plug (flybyODuKF)
   ------------------------------------------     --------------------------
   FilterState<State>                        <==  StateVector<Position<3>,
                                                              Velocity<3>>
   Dynamics<D, State>                        <==  two-body gravity lambda
   Measurement<M, State>                     <==  HeadingMeasurementModel
   SrukfSpec (State + ProcessNoiseCov)       <==  FlybyODuKFSpec
   SequentialFilter<Filter, Measurement>     <==  FlybyODuKFAlgorithm

``FlybyODuKFAlgorithm`` is the single composition root. It holds the
``SrukfInterface`` and the retained configuration, installs the dynamics
closure onto the estimator's ``dynamics`` member in ``reset()``, satisfies
``SequentialFilter`` via public ``timeUpdate`` / ``measurementUpdate``, owns
the ``measurement_queue<HeadingMeasurement, 1>``, exposes
``enqueueMeasurement`` and the single-call ``update(t0, t1)`` (which runs
``apply_sequential(measurements, *this, t0, t1)``), and packages the filter's
raw mean and sqrt-covariance into the POD ``FilterStateOutput`` /
``ResidualsOutput`` with unit unscaling applied.

Runtime view: one update window
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

A single ``update(t0, t1)`` drains the queue in time order; each step fans out
through the estimator facade into the functional core, the integrator, and the
supplied dynamics / measurement model:

::

   host adapter            FlybyODuKFAlgorithm                  filtering_core
   ------------            -------------------                  --------------
   read msg
   build HeadingMeasurement
   enqueueMeasurement ---> measurements.enqueue --------------> measurement_queue
   update(t0, t1) -------> apply_sequential(measurements, *this, t0, t1)
                            | per queued step, in time order:
                            |- algo.timeUpdate(dt) -----------> srukf.timeUpdate(dt)
                            |                                    `-> srukf::predict
                            |                                         `-> propagate -> rk4
                            |                                              `-> dynamics(t, x)
                            `- algo.measurementUpdate(m) -----> srukf.update(model)
                                                                 `-> srukf::update
                                                                      `-> m.model(x) / noise()
                                                                          / subtract(a, b)
   getState() / getLastResiduals() <-- algorithm packages its own mean +
                                       sqrtCovar + residuals into POD output
   write output payloads

``apply_sequential`` is what decides, per step, whether the next action is a
time update or a measurement update; the queue is just a container, and the
algorithm doesn't sequence that itself — it just exposes the
``SequentialFilter`` pair that ``apply_sequential`` calls. A filter family
that wants different scheduling (batch, iterated, particle resample,
out-of-order rewind) gets a different ``apply_*`` free template (constrained
on whatever concept that family needs) and reuses the same container.
The estimator behind the algorithm is likewise an internal detail: it only has
to make ``timeUpdate`` / ``measurementUpdate`` do the right thing, and the
queue, the scheduling function, the integrator, and the concepts don't depend
on which estimator is inside.

Why the pattern is generic
~~~~~~~~~~~~~~~~~~~~~~~~~~~~

- **The core knows nothing filter-specific.** ``measurement_queue``,
  ``propagate`` / ``rk4``, and the estimator engines are parametrized only by
  the ``Spec`` / ``State`` and driven entirely through concepts; they compile
  against any filter that satisfies the contracts.
- **A new filter is four plugs plus wiring.** Provide a ``State``
  (a ``StateVector<...>``), a ``Dynamics`` closure, one or more ``Measurement``
  models, and a ``Spec``; a single algorithm class wires them together,
  satisfies ``SequentialFilter`` via its ``timeUpdate`` / ``measurementUpdate``,
  owns a ``measurement_queue``, and exposes the host-facing surface. Nothing
  in the core is copied or edited.
- **Composition is by concept, not inheritance.** There are no virtual bases
  and no runtime dispatch; a type that fails a contract fails to compile at the
  call site with a concept diagnostic, and everything stays fixed-size and
  inlinable.
- **The estimator is itself a swappable plug.** Any estimator that presents the
  ``SequentialFilter`` pair (``timeUpdate`` / ``measurementUpdate``) can sit
  behind the algorithm, so the same queue and scheduling drive it unchanged;
  today that estimator is ``SrukfInterface<State>``.
- **Scheduling is a pluggable axis too.** ``measurement_queue`` is a generic
  time-ordered container; each scheduling policy is a free template
  (``apply_sequential`` today, ``apply_batch`` / ``apply_iterated`` /
  rewind-style / etc. as new filter families need them). An algorithm class
  picks the one that fits, and adding a new policy is a pure addition that
  doesn't touch the queue.

How to use a filter
-------------------

Configuring, resetting, and stepping ``FlybyODuKFAlgorithm`` directly (no
xmera):

.. code-block:: cpp

   using namespace filtering::flybyODuKF;

   FlybyODuKFAlgorithm algo;           // owns the SRUKF + measurement_queue
   algo.setMu(42828.314e9);            // m^3/s^2 (SI in, converted internally)
   algo.setUnitConversion(1e-3);       // work in km / km/s
   algo.setAlpha(0.02);
   algo.setBeta(2.0);
   algo.setProcessNoise(Q);            // 6x6
   algo.setInitialCovariance(P0);      // 6x6

   FlybyODuKFAlgorithm::State x0;
   x0.set<filtering::Position<3>>(r0);
   x0.set<filtering::Velocity<3>>(v0);
   algo.setInitialState(x0);

   algo.reset();                       // pushes config into the SRUKF, installs
                                       // dynamics, clears the queue

   HeadingMeasurement meas;
   meas.timeTag   = t1;
   meas.rhat_BN_N = heading;           // unit vector body -> target, frame N
   meas.covarN    = covariance;
   meas.valid     = true;

   // Queue-driven path: enqueue, then one drive call per window. update(t0, t1)
   // delegates to apply_sequential(measurements, *this, t0, t1), which
   // interleaves time and measurement updates against *this.
   algo.enqueueMeasurement(meas.timeTag, meas);
   algo.update(t0, t1);

   // Direct-stepping path: call the SequentialFilter pair yourself (this is
   // what apply_sequential calls under the hood, and what the gtest's
   // Propagation / MeasurementUpdates cases exercise).
   algo.timeUpdate(dt);                // predict
   algo.measurementUpdate(meas);       // fold in a measurement directly

   FilterStateOutput out = algo.getState();         // mean + sqrt-covariance (SI)
   ResidualsOutput   res = algo.getLastResiduals();  // pre/post-fit residuals

How to add a new filter
-----------------------

#. **Pick the state.** Compose it from component tags, e.g.
   ``using SunlineState = StateVector<Position<3>, Velocity<3>, Bias<3>>;``.
#. **Write the Spec.** A small struct with ``using State = ...;`` and
   ``using ProcessNoiseCov = Eigen::Matrix<double, N, N>;`` — that is all the
   ``SrukfSpec`` concept requires. Keep it in the header so the type is
   complete where ``SrukfInterface<Spec>`` is instantiated.
#. **Write the I/O types** (``*Types.h``) — POD structs for the measurement(s)
   in and the state/residuals out.
#. **Write the algorithm class** — the single composition root. Holds a
   ``SrukfInterface<State>``, all SRUKF
   configuration with its setter/getter implementations, ``reset()``, the
   ``SequentialFilter`` pair (``timeUpdate(dt)`` / ``measurementUpdate(m)``),
   the raw readouts (``getMean`` / ``getSqrtCovar`` / ``getCurrentTime``), a
   ``measurement_queue<Measurement, CAPACITY>``, plus
   ``enqueueMeasurement(timeTag, m)`` and the single-call ``update(t0, t1)``
   that runs ``apply_sequential(this->measurements, *this, t0, t1)`` (or a
   different ``apply_*`` if your filter family wants different scheduling).
   ``getState()`` / ``getLastResiduals()`` package the raw readouts into POD
   output with unit unscaling. Put the dynamics closure and measurement
   model in the ``.cpp``; they only need to *satisfy* ``Dynamics<D, State>``
   and ``Measurement<M, State>`` — no base class to inherit.
#. **Add the static library** under ``src/filtering_algorithms/<filter>`` and an
   ``add_subdirectory`` entry, linking ``filtering_core`` ``PUBLIC``.
#. **Write a gtest** (``_tests/``) that drives the algorithm class directly,
   with no xmera — config round-trip, direct ``timeUpdate`` / ``measurementUpdate``
   stepping, and the queue-driven path through ``update(t0, t1)``. See
   ``filtering_algorithms/flybyODuKF/_tests``.
#. **Write the host adapter** under ``fswAlgorithms/`` that marshals messages to
   and from the POD I/O types and links the algorithm library.

.. note::

   **Variable-size measurements** (e.g. a varying number of active coarse sun
   sensors) are handled with a fixed-capacity ``Eigen::Vector<double, MAX>`` and
   a runtime ``numActive`` count, sliced with ``.head()`` / ``.topRows()`` — not
   a dynamically sized Eigen type. ``FlybyODuKF`` does not exercise this; its
   measurement is a single fixed 3-vector.

The boundary rule
-----------------

``filtering_core`` and ``filtering_algorithms/*`` must never depend on xmera.
Concretely, none of these may appear in their sources or headers:

.. code-block:: text

   architecture/messaging        architecture/_GeneralModuleFiles/sys_model
   *MsgPayload                    ReadFunctor / Message
   : public SysModel

If you need any of those, it belongs in the host adapter, not the algorithm.

Deferred work
-------------

Two follow-on steps are intentionally out of scope of the initial extraction
but the shapes above are designed to make them mechanical:

- **Single precision.** The algorithm is ``double`` today. Templating the core
  on a ``Scalar`` type and instantiating ``float`` is a separate step (it
  surfaces Cholesky stability questions).
- **C shim.** Porting to ``fp32-fsw-xmera`` adds an ``extern "C"`` wrapper
  around the **algorithm** class (e.g. ``FlybyODuKFAlgorithm`` →
  ``flybyODuKFAlgorithm_c.{h,cpp}``) — opaque handle, row-major contract — so
  Adamant can call it. The algorithm is the right wrap target because its API
  is POD-facing: ``enqueueMeasurement(HeadingMeasurement)``,
  ``update(double, double)``, ``getState() → FilterStateOutput``,
  ``getLastResiduals() → ResidualsOutput``. The POD ``*Types.h`` structs are
  the data seam the shim mirrors as ``_c`` wrappers. The filter class stays
  internal to the algorithm library; the shim never sees it.
