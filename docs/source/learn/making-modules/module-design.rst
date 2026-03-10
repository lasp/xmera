.. _makingModules-1:

Module Design Considerations
============================

A basic Xmera module encapsulates some mathematical behavior of a spacecraft or the space environment. Information is passed between modules through the message passing interface. This interface consists of message objects that contain the data and provide output message connections. These are read by input message readers, which pull data from the outgoing message and return a copy of the data structure.

In shorthand, these are referred to as input and output message objects.

.. image:: ../../_images/static/qs-mm-1-0.svg
   :align: center

Message Connections
-------------------

The illustration above shows how a module can contain multiple input and output messages. Some messages may be required for a module to function properly, while others are optional. For example, consider the :ref:`mrpFeedback` module. It includes optional input messages that read in the reaction wheel states and configuration parameters. If these messages are connected, the control algorithm will incorporate their data. If they are not connected, the module defaults to a basic attitude feedback control that excludes the reaction wheel devices.

One Big Module or Several Small Modules
---------------------------------------

A common design question in modular software development is whether to implement functionality in one large module or divide it into smaller modules. The design goal should emphasize flexibility and reusability.

Consider the math involved in your module: Is it always used together as one unit, or could some of it be reused in other contexts? If the latter, it is recommended to split the logic into multiple smaller modules.

.. image:: ../../_images/static/qs-mm-1-1.svg
   :align: center

Take the :ref:`reactionWheelStateEffector` as an example. This module accepts an array of motor torques as input. However, some reaction wheel (RW) devices are controlled via analog or digital interfaces. This functionality is intentionally not included in the `reactionWheelStateEffector` so that the RW physics and the control interface can be modular and interchangeable.

The image above shows this design approach: RW physics is encapsulated in one module, while separate modules handle the digital interface and the motor torque behavior. With this structure, a Xmera simulation can either directly drive the RWs with motor torque commands or simulate higher fidelity by including the motor behavior and/or control interface modules.

Variable Number of Input or Output Messages
-------------------------------------------

In some cases, it is useful to design a module that can handle a variable number of input and/or output messages. In C++, this can be achieved using a ``std::vector`` of messages. In C, this is typically implemented using fixed-length arrays to avoid dynamic memory allocation.

Consider the :ref:`eclipse` module. It has a single input message providing the Sun’s location. Rather than limiting the module to computing eclipse conditions relative to a single planet, it is designed to handle eclipses involving multiple celestial bodies. This allows a spacecraft to transition from orbiting Earth to orbiting Mars in the same simulation, while still accounting for eclipses caused by both bodies.

When implementing variable-length message support, additional considerations are needed. For example, when adding a new planet state input to the `eclipse` module, the internal buffer of message data must also be expanded. To ensure consistency, this module uses the method ``addPlanetToModel()`` to add a new planet input. This method manages both the vector of input messages and their associated internal buffers.

Similarly, the `eclipse` module is capable of generating eclipse data for multiple spacecraft. This avoids the inefficiency of creating duplicate eclipse modules for each spacecraft. Instead, the module reads vectors of both planet and spacecraft state messages. For each spacecraft, a corresponding eclipse output message is generated.

To facilitate this, the ``addSpacecraftToModel()`` method is used. This method:

- Accepts a spacecraft state input message.
- Adds it to the internal vector of input messages.
- Expands the vector of internal buffer variables.
- Creates the associated spacecraft eclipse output message.

.. image:: ../../_images/static/qs-mm-1-2.svg
   :align: center

For an example of a C module handling a variable number of messages, see :ref:`navAggregate`.
