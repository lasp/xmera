
Base class for every integrator that advances the states of a :ref:`dynamicObject`.

To create a new integrator, inherit from this class and override ``integrate()``.  The method only has to
advance the states one time step, from ``currentTime`` to ``currentTime + timeStep``.

The dynamic object to integrate is passed in as an argument, so an integrator holds no reference to the object
it advances.  Working memory that would otherwise be allocated on every step should be held as a member and
reused, as :ref:`Rk4Integrator` does with its copies of the state vector.

An implementation drives the dynamic object through two interfaces.  ``DynamicObject::equationsOfMotion()``
evaluates the state derivatives at a given time.  The state manager ``DynamicObject::dynManager`` reads and
writes the states themselves, through ``getStateVector()``, ``updateStateVector()`` and ``propagateStateVector()``,
while ``StateVector::setDerivativesFrom()`` and ``StateVector::propagateState()`` accumulate the stage results.
