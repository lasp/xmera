.. _xmeraPrinciples-9:

Advanced: Using ``DynamicObject`` Xmera Modules
==================================================
Xmera modules such as :ref:`spacecraft` are members of
the ``DynamicObject`` class.  This means they still have the regular Xmera ``reset()`` and
``updateState()`` methods, but they also have a state machine and integrator build in as these
modules have to integrate internal ordinate differential equations (ODEs).

Xmera modules such as :ref:`spacecraft` and :ref:`spacecraftSystem` inherit from the ``DynamicObject`` class. These
modules include the standard ``reset()`` and ``updateState()`` methods, but also incorporate a built-in state machine
and numerical integrator to solve internal ordinary differential equations (ODEs).

A key feature of ``DynamicObject`` is the ability to synchronize integration across multiple instances of these modules.
This is particularly useful in scenarios where dynamics are coupled—such as two spacecraft interacting through
force or torque-based effectors.

Suppose you have two spacecraft instances, ``scObject`` and ``scObject2``, and you want their dynamic integration to be
synchronized. This can be achieved from Python using the following command::

    scObject.syncDynamicsIntegration(scObject2)

This call links the integration of ``scObject2`` to that of ``scObject``. Even if ``scObject`` is configured to run
in a task executing at 10 Hz using an RK4 integrator, and ``scObject2`` is in a separate task running at 1 Hz with an
RK2 integrator, the integration setup of the primary object (``scObject``) overrides the configuration of the secondary
synced object. As a result, both objects are integrated using the RK4 integrator at 10 Hz. However, the
``updateState()`` method of ``scObject2`` is still called at its task’s update rate of 1 Hz.

.. note::

    The ``syncDynamicsIntegration()`` method is not limited to linking two ``DynamicObject`` instances. The primary
    ``DynamicObject`` maintains a list of all synced objects. This allows you to synchronize the integration across
    any number of ``DynamicObject`` instances.

The integration algorithm is determined by the integrator assigned to the primary ``DynamicObject``. By default, this
is the Runge-Kutta 4 (RK4) method. Regardless of the integrator settings on secondary objects, the integration
configuration of the primary object takes precedence.
