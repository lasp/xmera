.. _filtering-architecture:

Filtering Architecture (filteringCore)
=======================================

.. note::

   This page describes the framework-agnostic Kalman-filter infrastructure
   under ``src/filteringCore`` and the per-filter algorithm libraries under
   ``src/filtering_algorithms``. It explains how the pieces relate, how to use
   a filter, and how to add a new one. The first filter built on this
   infrastructure is :ref:`flybyODuKF`.

Why this exists
---------------

The filtering infrastructure splits that responsibility into three layers:

#. a **framework-agnostic core** (``filteringCore``) — the reusable filter
   machinery, depending only on Eigen and the C++ standard library;
#. a **per-filter algorithm library** (``filtering_algorithms/<filter>``) — the
   concrete dynamics, measurement model, and a plain C++ class that owns the
   filter state;
#. a **thin host adapter** (under ``fswAlgorithms/``) — the only layer that
   knows about xmera messaging and ``SysModel``.

The boundary is enforced by the build: ``filteringCore`` and
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
   |   - FlybyODuKFAlgorithm   (stateful, plain C++ class)        |
   |   - *Specs.h / *Types.h   (State, dynamics, POD I/O types)   |
   |   - measurement model(s) (in the .cpp)                       |
   +----------------------------+--------------------------------+
                                | built on
                                v
   +-------------------------------------------------------------+
   | filteringCore   (header-only INTERFACE library)            |
   |   - StateVector<Components...> + component tags             |
   |   - concepts: FilterState, Measurement, Dynamics, ...       |
   |   - DynamicsModel / rk4 / propagate                         |
   |   - measurement_queue                                       |
   |   - SRUKF functional core + SRuKF<Spec> facade     |
   +-------------------------------------------------------------+
                  depends only on Eigen + the C++ stdlib

filteringCore
~~~~~~~~~~~~~~~

Header-only ``INTERFACE`` library (``src/filteringCore/include/filteringCore``).
Headers are included as ``<filteringCore/...>``.

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

``dynamicsModel.hpp``
   ``DynamicsModel<State>`` (a ``std::function`` alias) plus ``rk4()`` and
   ``propagate()`` — concept-constrained RK4 integration usable on any
   ``LinearlyCombinable`` state.

``kalmanFilter.hpp``
   The ``SequentialFilter<F, M>`` concept — the contract a Kalman-style filter
   satisfies (``timeUpdate(dt)`` + ``measurementUpdate(m)``).

``measurementQueue.h``
   ``measurement_queue<Measurement, CAPACITY>`` — a bounded, time-ordered
   container (``enqueue`` / ``popEarliest`` / ``clear`` / ``isFull`` /
   ``isEmpty``) plus a persistent ``getTimeOfLastMeasurement()`` /
   ``setTimeOfLastMeasurement()`` pair: the time of the last measurement
   the scheduler folded into the filter. Scheduling itself is a free
   template (see ``kalmanFilter.hpp``).

``kalmanFilter.hpp``
   The ``SequentialFilter<F, M>`` concept plus
   ``apply_sequential(queue, filter, callTime)`` — the canonical scheduler
   that interleaves time and measurement updates relative to the queue's
   last-measurement time. Alternative scheduling styles (batch, iterated, ...)
   drop in as new ``apply_*`` free templates without touching the queue.

``srukf.hpp``
   The square-root UKF (van der Merwe & Wan, ICASSP 2001) in two layers:

   - a **functional core** — ``SrukfStorage<State>`` (plain data, all
     fixed-size) and free functions ``srukf::reset``,
     ``srukf::timeUpdate<State, D>``, ``srukf::measurementUpdate<State, M>``;
   - a **stateful facade** — ``SRuKF<State, Dyn>``, which holds a
     ``SrukfStorage`` and a settable ``dynamics`` member and exposes
     ``reset()`` / ``timeUpdate(dt)`` / ``measurementUpdate<M>(m)`` plus
     setters/getters and a ``getStateAtLastMeasurement()`` /
     ``setStateLastMeasurement()`` pair for the rolling last-measurement
     state.

   ``timeUpdate(dt)`` always **rewinds to the last-measurement state** before
   propagating, so it is idempotent in ``dt`` — calling it twice with the
   same elapsed time produces the same posterior. Only
   ``measurementUpdate`` advances the last-measurement state.

filtering_algorithms/<filter>
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

A ``STATIC`` library per filter, depending only on ``filteringCore``. For
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

Every filter built on ``filteringCore`` is the *same machine with a few holes
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
     - ``SRuKF<State, Dyn>``, ``propagate``
     - ``StateVector<Position<3>, Velocity<3>>``
   * - How the state evolves
     - ``Dynamics<D, State>``
     - ``rk4`` / ``propagate`` (in ``srukf::timeUpdate``)
     - two-body gravity closure
   * - How the state is observed
     - ``Measurement<M, State>``
     - ``srukf::measurementUpdate``
     - ``HeadingMeasurementModel``
   * - The drivable filter
     - ``SequentialFilter<F, M>`` (``timeUpdate`` + ``measurementUpdate``)
     - ``apply_sequential`` (free template)
     - ``FlybyODuKFAlgorithm``
   * - When to time-update vs. measurement-update
     - — (concrete, not a concept)
     - ``measurement_queue`` (container) + ``apply_sequential`` (free function)
     - ``FlybyODuKFAlgorithm`` owns one ``measurement_queue<HeadingMeasurement, 1>`` and runs ``apply_sequential(measurements, *this, callTime)`` from its ``update``

The first three rows are the *plugs* a new filter must provide; everything in
the "Core consumer" column is reused unchanged.

Static view: concepts as sockets
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Each concept is a socket in the core; a concrete type plugs in by *satisfying*
it — no inheritance, checked at compile time:

::

   generic socket (concept in filteringCore)    concrete plug (flybyODuKF)
   ------------------------------------------     --------------------------
   FilterState<State>                        <==  StateVector<Position<3>,
                                                              Velocity<3>>
   Dynamics<D, State>                        <==  two-body gravity lambda
   Measurement<M, State>                     <==  HeadingMeasurementModel
   SequentialFilter<Filter, Measurement>     <==  FlybyODuKFAlgorithm

``FlybyODuKFAlgorithm`` is the single composition root. It holds the
``SRuKF`` and the retained configuration, installs the dynamics
closure onto the estimator's ``dynamics`` member in ``reset()``, satisfies
``SequentialFilter`` via public ``timeUpdate`` / ``measurementUpdate``, owns
the ``measurement_queue<HeadingMeasurement, 1>``, exposes
``enqueueMeasurement`` and a single-call ``update(t0, t1)`` (which runs
``apply_sequential(measurements, *this, t1)``), and packages the filter's
state and covariance into the POD ``FilterStateOutput`` /
``ResidualsOutput``. If the filter conditions the state into internal units
for numerical stability (e.g. m → km), it pre-scales the inputs to the SRUKF
in ``reset()`` and unscales the outputs in ``getState()`` — the SRUKF itself
is unit-agnostic.

Runtime view: one update window
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

A single ``update(t0, t1)`` drains the queue in time order; each step fans out
through the estimator facade into the functional core, the integrator, and the
supplied dynamics / measurement model:

::

   host adapter            FlybyODuKFAlgorithm                  filteringCore
   ------------            -------------------                  --------------
   read msg
   build HeadingMeasurement
   enqueueMeasurement ---> measurements.enqueue --------------> measurement_queue
   update(t0, t1) -------> apply_sequential(measurements, *this, t1)
                            | per queued step, in time order:
                            |- algo.timeUpdate(dt) -----------> srukf.timeUpdate(dt)
                            |                                    `-> srukf::timeUpdate
                            |                                         `-> propagate -> rk4
                            |                                              `-> dynamics(t, x)
                            `- algo.measurementUpdate(m) -----> srukf.measurementUpdate(model)
                                                                 `-> srukf::measurementUpdate
                                                                      `-> m.model(x) / noise()
                                                                          / subtract(a, b)
   getState() / getLastResiduals() <-- algorithm packages its own state +
                                       covariance + residuals into POD output
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

Time bookkeeping: queue ↔ SRUKF
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

The scheduler tracks the time of the most recent measurement in two places
that stay in lockstep:

- ``queue.getTimeOfLastMeasurement()`` — the *time* of the last measurement
  ``apply_sequential`` folded in; persisted across calls.
- ``srukf.getStateAtLastMeasurement()`` — the *state* (and the matching
  covariance / sqrtCovar) at that same time; updated inside
  ``srukf::measurementUpdate``.

``timeUpdate(dt)`` rewinds the working ``state`` / ``covariance`` /
``sqrtCovar`` to the last-measurement values, then propagates by ``dt``. So
``dt`` is always the elapsed time since the last measurement, not the elapsed
time since the previous ``timeUpdate`` call.

Walkthrough of ``apply_sequential(queue, filter, callTime)``::

   Timeline (queue contents):

                       m1 (t=2)        m2 (t=4)             callTime (t=7)
                          ▼               ▼                    │
    ──•───────────────────•───────────────•────────────────────•─────→  t
      │
      tLast := queue.getTimeOfLastMeasurement()   (e.g. t=0)

   apply_sequential:
     tLast = queue.getTimeOfLastMeasurement()                    // = 0
     for each m in queue (earliest first):
         if m.timeTag < tLast: continue                          // drop late arrivals
         filter.timeUpdate(m.timeTag - tLast)                    // rewind+propagate
         filter.measurementUpdate(m)                             // updates last-meas state
         tLast = m.timeTag                                       // local cursor
     if tLast < callTime:
         filter.timeUpdate(callTime - tLast)                     // for OUTPUT only
     queue.setTimeOfLastMeasurement(tLast)                       // persist

After the loop above runs against m1 then m2:

- inside the SRUKF: ``stateLastMeasurement`` = post-update state at t=4;
- on the queue: ``getTimeOfLastMeasurement() == 4``;
- the final ``filter.timeUpdate(callTime - 4) == timeUpdate(3)`` advances
  ``state``/``covariance`` to t=7 **without** moving the last-measurement
  state. The output messages read this propagated value.

On the next call, if a new measurement arrives at t=10, the scheduler issues
``filter.timeUpdate(10 - 4) == timeUpdate(6)`` — the SRUKF rewinds to its
t=4 last-measurement state and propagates 6 s forward, **not** 3 s past the
previously-output t=7 state. No drift accumulates from the output-only
propagation.

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
  today that estimator is ``SRuKF<State>``.
- **Scheduling is a pluggable axis too.** ``measurement_queue`` is a generic
  time-ordered container; each scheduling policy is a free template
  (``apply_sequential`` today, ``apply_batch`` / ``apply_iterated`` /
  rewind-style / etc. as new filter families need them). An algorithm class
  picks the one that fits, and adding a new policy is a pure addition that
  doesn't touch the queue.

Heterogeneous measurements
~~~~~~~~~~~~~~~~~~~~~~~~~~~~

A multi-sensor filter (e.g. sunline = gyro + coarse sun sensors, inertial
attitude = star tracker + gyro) consumes more than one *kind* of measurement
on one timeline. No core change is needed for this — the core is
kind-agnostic:

- ``measurement_queue``, ``apply_sequential``, and ``SequentialFilter`` all
  operate over an arbitrary ``Measurement`` type.
- ``srukf::measurementUpdate<M>`` is templated per measurement model, and
  each model carries its own ``M::size``, so different observation
  dimensions already work (the flyby heading model is size 3; the toy test
  below also drives a size-1 model through the same path).

``std::variant`` is freestanding on the target toolchain (GCC 14+ / C++26
P2407, ``__cpp_lib_freestanding_variant``), so it is an appropriate closed-set
mechanism here. The filter that knows its kinds names the set as a
``std::variant`` and dispatches with ``std::visit`` — fixed-size, no heap, no
virtual:

.. code-block:: cpp

   using Measurement = std::variant<HeadingMeasurement, GyroMeasurement>;
   filtering::measurement_queue<Measurement, CAP> measurements;   // one timeline

   void measurementUpdate(Measurement const& m) {                 // SequentialFilter
       std::visit([this](auto const& meas){ this->applyMeasurement(meas); }, m);
   }
   // one private overload per kind: POD -> model -> srukf.update(model)
   void applyMeasurement(HeadingMeasurement const&);
   void applyMeasurement(GyroMeasurement const&);

Because every kind shares the one queue timeline, ``apply_sequential``
interleaves them in time order for free — no per-kind sub-queues, no
hand-rolled merge. The variant is over the **input PODs**; each
``applyMeasurement`` overload builds that kind's **model** (the
``Measurement<M, State>`` object) and calls ``srukf.update`` — the same
POD-vs-model split the single-kind filter uses.

A worked, compiling reference (real SRUKF, two kinds of observation sizes 3
and 1) lives in
``filteringCore/_tests/test_heterogeneous_measurements.cpp``.

C-shim note: a ``std::variant`` doesn't cross a C ABI. The shim exposes one
``enqueue_<kind>()`` per kind — it already mirrors each POD as a ``_c`` struct
— and constructs the variant on the C++ side.

(FlybyODuKF stays single-kind: it observes only a heading. The variant pattern
lands for real when the multi-sensor filters are ported.)

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
#. **Write the I/O types** (``*Specs.h`` / ``*Types.h``) — POD structs for
   the measurement(s) in and the state/residuals out, plus the dynamics
   functor that satisfies ``Dynamics<D, State>``.
#. **Write the algorithm class** — the single composition root. Holds a
   ``SRuKF<State, Dyn>``, the retained configuration with
   setter/getter implementations, ``reset()``, the ``SequentialFilter``
   pair (``timeUpdate(dt)`` / ``measurementUpdate(m)``), the readouts
   (``getState`` / ``getCovariance`` / per-kind residuals), a
   ``measurement_queue<Measurement, CAPACITY>``, plus a single-call
   ``update(...)`` that runs
   ``apply_sequential(this->measurements, *this, callTime)`` (or a
   different ``apply_*`` if your filter family wants different scheduling).
   The ``update`` signature is filter-specific — flyby uses
   ``update(t0, t1)``; sunline uses ``update(currentSeconds, CssData,
   RateData)`` and returns a bundled ``SunlineSRuKFOutput``. Put the
   measurement model(s) in the ``.cpp`` — they only need to *satisfy*
   ``Measurement<M, State>``, no base class to inherit. If your filter needs
   unit conditioning, pre-scale inputs to the SRUKF in ``reset()`` and
   un-scale outputs in the readouts — the SRUKF itself is unit-agnostic.
#. **Add the static library** under ``src/filtering_algorithms/<filter>`` and an
   ``add_subdirectory`` entry, linking ``filteringCore`` ``PUBLIC``.
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

``filteringCore`` and ``filtering_algorithms/*`` must never depend on xmera.
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
