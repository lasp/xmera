.. _xmera_module_interface:

Modules
=======

Overview
--------

In the Xmera simulation framework, a **module** is a self-contained component that performs a specific simulation
task, such as dynamics modeling, sensor processing, or control logic. Modules inherit from the `SysModel` interface
and follow a common structure to ensure they can be scheduled and run within a simulation task.

Requirements
------------

Every module must:

- Inherit from `SysModel`
- Implement the following virtual methods:

  - ``void Reset(uint64_t CurrentSimNanos)``
  - ``void UpdateState(uint64_t CurrentSimNanos)``

Example
-------

.. code-block:: cpp

    #include "architecture/_GeneralModuleFiles/sys_model.h"
    #include <iostream>

    class MyExampleModule : public SysModel {
    public:

        MyExampleModule() {
            this->modelTag = "MyExampleModule";
        }

        void reset(uint64_t currentSimNanos) override {
            std::cout << "[" << modelTag << "] Reset at t=" << currentSimNanos << std::endl;
        }

        void updateState(uint64_t currentSimNanos) override {
            std::cout << "[" << modelTag << "] Updating at t=" << currentSimNanos << std::endl;
        }
    };

Input/Output
------------

Modules typically use the Xmera messaging system to receive inputs and publish outputs.

.. code-block:: cpp

    ReadFunctor<ExampleMsgPayload> inputMsg;
    Message<ExampleOutputMsgPayload> outputMsg;

    void reset(uint64_t currentSimNanos) override {
        // Optional: validate message is linked
    }

    void update(uint64_t currentSimNanos) override {
        ExampleMsgPayload input = this->inputMsg();
        ExampleOutputMsgPayload output = {};

        // Compute results using `input`
        this->outputMsg.write(output, this->moduleID, currentSimNanos);
    }
