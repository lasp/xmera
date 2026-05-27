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
   |   - EkfInterface                                            |
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
   ``measurement_queue<Measurement, CAPACITY>`` — a bounded, time-ordered queue,
   and ``applyToFilter()`` which interleaves time and measurement updates.

``srukf_interface.hpp``
   The square-root UKF in two layers:

   - a **functional core** — ``SrukfStorage<State>`` (plain data, all
     fixed-size) and free functions ``srukf::reset<Spec>``,
     ``srukf::predict<Spec, D>``, ``srukf::update<Spec, M>``;
   - a **stateful facade** — ``SrukfInterface<Spec>``, which holds a
     ``SrukfStorage`` and a settable ``dynamics`` member and exposes
     ``reset()`` / ``predict(dt)`` / ``update<M>(m)`` plus setters/getters.

``ekf_interface.hpp``
   ``EkfInterface<STATE_SIZE, MAX_MEAS_SIZE>`` — the fixed-size extended/classical
   Kalman filter, also concept-based (no virtual measurement base).

filtering_algorithms/<filter>
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

A ``STATIC`` library per filter, depending only on ``filtering_core``. For
``flybyODuKF`` it contains:

- ``flybyODuKFTypes.h`` — POD input/output structs (``HeadingMeasurement``,
  ``FilterStateOutput``, ``ResidualsOutput``) the host adapter marshals to and
  from. These are the seam the future C shim wraps.
- ``flybyODuKFAlgorithm.h`` — the ``FlybyODuKFSpec`` (just the ``State`` and
  ``ProcessNoiseCov`` typedefs the ``SrukfSpec`` concept requires) and the
  ``FlybyODuKFAlgorithm`` class: setters/getters for tunables, ``reset()`` /
  ``predict(dt)`` / ``update(...)``, and value-returning readout.
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
``measurement_queue`` and decides per step — via the queue's ``applyToFilter``
— whether to ``timeUpdate`` (predict) or ``measurementUpdate``. The adapter
then writes the algorithm's ``getState()`` / ``getLastResiduals()`` back into
the output payloads.

How to use a filter
-------------------

A filter is configured, reset, then stepped. Using ``FlybyODuKFAlgorithm``
directly (no xmera):

.. code-block:: cpp

   using namespace filtering::flybyODuKF;

   FlybyODuKFAlgorithm filter;
   filter.setMu(42828.314e9);          // m^3/s^2 (SI in, converted internally)
   filter.setUnitConversion(1e-3);     // work in km / km/s
   filter.setAlpha(0.02);
   filter.setBeta(2.0);
   filter.setProcessNoise(Q);          // 6x6
   filter.setInitialCovariance(P0);    // 6x6

   FlybyODuKFAlgorithm::State x0;
   x0.set<filtering::Position<3>>(r0);
   x0.set<filtering::Velocity<3>>(v0);
   filter.setInitialState(x0);

   filter.reset();                     // pushes config into the SRUKF, sets dynamics

   // Lowest level: drive the filter directly (this is what the queue calls).
   filter.timeUpdate(dt);              // predict step

   HeadingMeasurement meas;            // measurement update
   meas.rhat_BN_N = heading;           // unit vector body -> target, frame N
   meas.covarN    = covariance;
   meas.valid     = true;
   filter.measurementUpdate(meas);

   // Host-adapter level: enqueue measurements, then one drive call per window.
   // update(t0, t1) interleaves time and measurement updates from the queue.
   filter.enqueueMeasurement(meas.timeTag, meas);
   filter.update(t0, t1);

   FilterStateOutput out = filter.getState();        // mean + sqrt-covariance (SI)
   ResidualsOutput   res = filter.getLastResiduals(); // pre/post-fit residuals

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
#. **Write the algorithm class** holding a ``SrukfInterface<Spec>`` (or
   ``EkfInterface<...>``), with setters, ``reset``/``predict``/``update``, and
   readout. Put the dynamics closure and measurement model in the ``.cpp``;
   they only need to *satisfy* ``Dynamics<D, State>`` and
   ``Measurement<M, State>`` — no base class to inherit.
#. **Add the static library** under ``src/filtering_algorithms/<filter>`` and an
   ``add_subdirectory`` entry, linking ``filtering_core`` ``PUBLIC``.
#. **Write a gtest** (``_tests/``) that drives the algorithm class directly,
   with no xmera. See ``filtering_algorithms/flybyODuKF/_tests``.
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
  (opaque handle, POD wrapper structs, row-major contract) around the algorithm
  class so Adamant can call it. The POD ``*Types.h`` already define that seam.
