.. _xmeraPrinciples-9:

Advanced: Using ``DynamicObject`` Xmera Modules
==================================================
Xmera modules such as :ref:`spacecraft` inherit from the ``DynamicObject`` class. These modules include the
standard Xmera ``reset()`` and ``updateState()`` methods, but they also carry a state manager and a numerical
integrator, because they have to solve internal ordinary differential equations (ODEs).

Each ``DynamicObject`` integrates its own states, and those of its attached state and dynamic effectors, during its
own ``updateState()`` call.  The integration method is the fixed step Runge-Kutta 4 (RK4) scheme, which
:ref:`spacecraft` sets up when it is created.  If several ``DynamicObject`` instances are added to a simulation,
each one is integrated in turn as its task runs, at the update rate of the task it belongs to.

See :ref:`creatingDynObject` for how to write a Xmera module that inherits from ``DynamicObject``.
