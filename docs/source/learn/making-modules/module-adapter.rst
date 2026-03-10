.. _adapter_pattern_xmera:

Modules in C or Rust
====================

Overview
--------

The **Adapter Pattern** is a structural design pattern used to bridge the gap between incompatible interfaces.
In the context of the Xmera simulation framework, the adapter pattern can be used to integrate libraries
written in **C**, **Rust**, **Fortran** (or any library with a C ABI) into the **C++-based SysModel architecture**
of Xmera.

This is particularly useful when:

* A third-party or legacy model exists in another language.
* You want to reuse that code without rewriting it in C++.
* The library has a stable and tested API that you want to integrate with a Xmera simulation.

Xmera SysModel Requirements
------------------------------

All Xmera modules must inherit from the `SysModel` interface. As a result, a module is implemented as an *adapter*
between Xmera and your compiled library's API.

External Library Example
------------------------

C Library Example
^^^^^^^^^^^^^^^^^

.. code-block:: c

    // sensor_lib.h
    typedef struct {
        double value;
    } SensorState;

    void sensor_init(SensorState* state);
    void sensor_update(SensorState* state, double input);

Rust Library (compiled with ``#[no_mangle]``)
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. code-block:: rust

    #[repr(C)]
    pub struct SensorState {
        pub value: f64,
    }

    #[no_mangle]
    pub extern "C" fn sensor_init(state: *mut SensorState) {
        unsafe { (*state).value = 0.0; }
    }

    #[no_mangle]
    pub extern "C" fn sensor_update(state: *mut SensorState, input: f64) {
        unsafe { (*state).value += input; }
    }

C++ Adapter Implementation
--------------------------

You can write a module class that acts as an adapter between the Xmera framework and the external code:

.. code-block:: cpp

    #include "architecture/_GeneralModuleFiles/sys_model.h"
    #include "sensor_lib.h"  // C or Rust header

    class SensorAdapter : public SysModel {
    public:
        std::string ModelTag = "SensorAdapter";
        SensorState sensorState;

        SensorAdapter() {
            sensor_init(&sensorState);
        }

        void reset(uint64_t currentSimNanos) override {
            sensor_init(&sensorState);
        }

        void update(uint64_t currentSimNanos) override {
            double input = 42.0;  // Example input
            sensor_update(&sensorState, input);
            // Use sensorState.value to write to a Xmera message
        }
    };

You can then use it like any other Xmera module.

Advantages
----------

.. list-table::
   :widths: 25 75
   :header-rows: 1

   * - Feature
     - Description
   * - Language Compatibility
     - Enables Rust/C/Fortran code to run in a C++ simulation
   * - Code Reuse
     - Avoid rewriting validated models
   * - Encapsulation
     - Keeps external code isolated from Xmera internals
   * - Maintainability
     - You can upgrade the external lib without changing Xmera

Tips for Rust Integration
-------------------------

When using Rust, generate a C-compatible header file with ``cbindgen``:

.. code-block:: bash

    cbindgen --crate my_rust_lib --output sensor_lib.h

Make sure to compile your Rust crate with ``--crate-type cdylib`` so it exposes C symbols.

Summary
-------

The Adapter Pattern is a robust solution for plugging external code into Xmera:

* ✅ Create a C++ adapter that implements `SysModel`
* ✅ Link and call the C or Rust library via a C-compatible API

This allows you to extend Xmera simulations with custom physics models, hardware simulators, or control systems
written in any compatible language.
