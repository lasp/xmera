.. _xmeraPrinciples-0:

What is Xmera
================

Motivation
----------

The purpose of the Xmera astrodynamics simulation framework is to enable sophisticated spacecraft
simulations to be rapidly created from a series of pre-built modules. Consider a deep space mission where
the spacecraft must perform a series of science, communication, station keeping, and orbit maneuver tasks.
Such operations require not only the modeling of spacecraft orbital and attitude dynamics, but also
sub-components such as reaction wheels, thrusters, flexible panels, and fuel slosh. Beyond the mechanical
components, the spacecraft also contains algorithms that determine orientation, perform closed-loop maneuvers,
process sensor data, etc. Depending on the depth of analysis, additional components such as batteries, data
storage units, or communication devices may also need to be modeled.

.. image:: ../../_images/static/qs-spacecraft.jpg
   :align: center
   :scale: 50 %

.. sidebar:: Framework

   Why is it a framework? Xmera by itself is not a program that simulates a spacecraft. Instead, it provides
   the building blocks—a software framework—used to build spacecraft mission simulations.

The purpose of Xmera, or BSK for short, is to provide an open-source, modular spacecraft simulation
environment that supports a wide range of numerical simulations. The atomic spacecraft behaviors and components
are encapsulated into modules, which use a message-passing interface to interact with each other.

.. image:: ../../_images/static/qs-bsk-lego.jpg
   :align: center
   :scale: 50 %

In essence, think of BSK as providing spacecraft LEGO blocks that users assemble into the desired spacecraft
behavior. The benefits of this approach include:

- Reuse of spacecraft simulation code, avoiding the need to re-create common features.
- Encapsulation of complex components into self-contained modules, so users only need to understand how to use
  the module, not its internal math.
- The ability to swap out individual components without affecting the rest of the simulation.
- No need for users to auto-generate or manually validate simulation code—modules are pre-developed and tested.

From Modules to Spacecraft
--------------------------

BSK modules have input and output message connections that are key to assembling components into a functioning
spacecraft simulation. These messages provide a structured method for sharing information across the simulation.

A module may have one or more input message connections. Some input messages are optional and modify the module's
behavior when connected. For example, a feedback control module may use a standard control approach when no
reaction wheel message is connected, but will incorporate the reaction wheel data if such a message is available.

.. image:: ../../_images/static/qs-bsk-0.svg
   :align: center

The content of a message is stored in an output message object. These message objects are typically embedded
within a BSK module but can also exist independently. Connecting output messages to input messages links modules
together, creating complex spacecraft behavior from modular, reusable components.

Module Behavior Testing
-----------------------

How do we know that the BSK modules are working as intended?

Each BSK module includes a ``_UnitTest`` folder containing Python scripts that test the module's functionality.
This ensures that the module behaves as expected in isolation.

Additionally, the ``src/examples`` directory contains integrated simulation scripts that assemble modules into
sample spacecraft simulations. These are also tested to verify that the integrated simulation functions properly.

In summary:

- Unit tests confirm that individual modules are working correctly.
- Integrated tests verify that module connections and interactions behave as intended.

Thanks to this robust testing framework, Xmera can be developed and extended with confidence that existing
functionality remains intact.
