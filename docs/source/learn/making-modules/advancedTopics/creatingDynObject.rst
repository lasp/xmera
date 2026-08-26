.. _creatingDynObject:

Creating ``DynamicObject`` Xmera Modules
===========================================

Xmera modules that inherit from the class :ref:`dynamicObject` are still regular Xmera modules
that have the typical ``selfInit()``, ``reset()`` and ``updateState()`` methods.  However, they also contain
a state engine called ``dynManager`` of the class ``DynParamManager``, as well as an integrator
pointer called ``integrator`` of class ``StateVecIntegrator``.  :ref:`spacecraft` is an example of
a Xmera module that is also inheriting from the ``DynamicObject`` class.

In the spacecraft ``updateState()`` method the ``DynamicObject::integrateState()`` method is called.
This call integrates all the registered spacecraft states, as well as all the connect state
and dynamic effectors, to the next time step using the connected integrator.

The ``initializeDynamics()`` virtual method must be defined in the ``DynamicObject`` subclass.
It typically performs the required setup steps, including registering the ODE states that are
to be integrated.

The ``DynamicObject`` class contains two virtual methods ``preIntegration()`` and ``postIntegration()``.
The ``DynamicObject`` subclass must define what steps are to be completed before the integration step,
and what post-integrations must be completed.  For example, with :ref:`spacecraft` the pre-integration
process determines the current time step to be integrated and stores some values used.  In the post-integration
step the MRP spacecraft attitude states are checked to not have a norm larger than 1 and the conservative DV
component is determined.

Xmera modules that are a subclass of ``DynamicObject`` are not restricted to mechanical integration
scenarios as with the spacecraft example.  Any module that has to solve a set of ordinary differential
equations can inherit from ``DynamicObject``.  See :ref:`xmeraPrinciples-9` for a discussion of how these
modules behave in a simulation.
