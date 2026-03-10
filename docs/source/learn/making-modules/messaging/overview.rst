.. _messaging-overview:
.. _xmera_messaging:

Messaging System Overview
=========================

What Is the Messaging System?
-----------------------------

The Xmera messaging system is a lightweight, strongly-typed, in-memory data
exchange mechanism for high-performance simulation.  It lets loosely coupled
modules communicate by passing structured messages through explicitly defined
input and output interfaces, without needing to know about each other's
internals.

Core Concepts
-------------

**Payloads** -- A payload is a plain C or C++ ``struct`` that defines the data
exchanged between modules (e.g. spacecraft states, control commands, sensor
readings).  Every message carries exactly one payload type.

**Output messages** (``Message<T>``) -- Declared by the module that *produces*
data.  The ``write()`` method stores a payload into the message's internal
buffer.

**Input messages** (``ReadFunctor<T>``) -- Declared by the module that
*consumes* data.  Calling the read functor returns a deep copy of the latest
payload written to the connected output message.

**Buffering** -- Each input message holds an independent copy of the last
written value.  This guarantees that every module sees a consistent snapshot of
its inputs during a simulation step, regardless of evaluation order.

**Type safety** -- The template parameter ``T`` is checked at compile time.  If
the payload type of an input message does not match the output it is connected
to, the compiler will reject the code.

Message Lifecycle
-----------------

1. **Declare** -- A module declares output messages (``Message<T>``) and input
   messages (``ReadFunctor<T>``) as public member variables in its header.

2. **Connect** -- Before the simulation starts, the user connects an input to
   an output.  In Python this looks like::

       moduleB.someInMsg.subscribeTo(moduleA.someOutMsg)

3. **Write** -- During ``updateState()``, the producing module fills a local
   payload buffer and writes it to the output message:

   .. code-block:: cpp

       SomeMsgPayload buf{};
       buf.value = 42.0;
       this->someOutMsg.write(&buf, this->moduleID, currentSimNanos);

4. **Read** -- The consuming module reads the input message by calling the read
   functor:

   .. code-block:: cpp

       SomeMsgPayload data = this->someInMsg();

Type Safety
-----------

All message reads and writes are checked at compile time through C++ templates.
If you accidentally connect or copy between mismatched payload types, the
compiler will produce an error -- there is no way to silently exchange
incompatible data at runtime.

Advantages
----------

- **Decoupling** -- Modules interact only through messages, making them
  independently testable and reusable across simulations.
- **Determinism** -- Each module operates on a frozen snapshot of its inputs
  during a time step, so evaluation order does not affect results.
- **Scalability** -- Adding a new module to a simulation requires only
  connecting its messages; no existing code needs to change.

Quick Example
-------------

.. code-block:: cpp

    // In the module header
    Message<ControlCmdPayload>      controlOutMsg;
    ReadFunctor<StateEstimatePayload> stateInMsg;

    // In updateState()
    if (this->stateInMsg.isLinked()) {
        StateEstimatePayload state = this->stateInMsg();

        ControlCmdPayload cmd{};
        cmd.torque = computeControl(state);
        this->controlOutMsg.write(&cmd, this->moduleID, currentSimNanos);
    }

Next Steps
----------

- :ref:`messaging-objects` -- detailed API for ``Message<T>`` and
  ``ReadFunctor<T>``
- :ref:`messaging-creating-types` -- how to define new message payload structs
