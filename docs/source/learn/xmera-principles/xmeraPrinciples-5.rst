.. _xmeraPrinciples-5:

Creating Stand-Alone Messages
=============================

.. sidebar:: Source Code

    The Python code shown below can be downloaded :download:`here </../../docs/source/code-samples/xmera-5.py>`.

In the previous example, messages embedded within modules were connected to form data pathways. However, certain use
cases require stand-alone messages that are created independently of any specific module. These are often used by
flight algorithm modules that rely on messages providing spacecraft configuration details, such as mass and inertia,
or actuator setups like thrusters or reaction wheels.

For example, when writing a unit test for a module, the test should ideally only execute the module being tested. All
necessary input messages should be created as independent stand-alone messages. This avoids dependencies on outputs
from other modules and makes the module test fully self-contained.

This tutorial demonstrates how to create and use a stand-alone message, and how to connect it to a module input. It
also shows how the simulation can be paused, message values changed, and then resumed.

.. image:: ../../_images/static/qs-bsk-5.svg
   :align: center

Creating a Stand-Alone Message
------------------------------

To create a stand-alone message, you must first define its payload—the data container for the message content. Suppose
you are working with a message type `someMsg`. The payload container is created using:

::

    msgData = messaging.someMsgPayload()

This creates an instance of the Python class that represents the message structure. Initially, all fields are set to
zero. You can modify any of these fields as needed. For example:

::

    msgData.variable = ...

Next, you create the message object and write the payload to it:

::

    msg = messaging.someMsg()
    msg.write(msgData)

These steps can be combined into one line for convenience:

::

    msg = messaging.someMsg().write(msgData)

In the example script below, a stand-alone message is created and connected to a module input.

.. literalinclude:: ../../code-samples/xmera-5.py
   :language: python
   :linenos:
   :lines: 18-

The simulation is initially run for 10 seconds. Afterward, the contents of the stand-alone message are updated. There
is no need to recreate the message object—it remains valid and connected to any module inputs. Only the data payload
needs to be updated.

The simulation’s stop time is then extended by another 10 seconds (for a total of 20 seconds), and execution resumes.
The updated results are reflected in the module's output message, which is plotted below.

.. image:: /_images/Scenarios/bsk-5.svg
   :align: center
