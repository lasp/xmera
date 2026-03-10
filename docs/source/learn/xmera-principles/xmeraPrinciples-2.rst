.. raw:: html

    <iframe width="560" height="315" src="https://www.youtube.com/embed/onv0jUOhBNM" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture" allowfullscreen></iframe>

.. _xmeraPrinciples-2:

Adding Xmera Modules
=======================

.. sidebar:: Source Code

    The Python code shown below can be downloaded :download:`here </../../docs/source/code-samples/xmera-2.py>`.

Now that we understand what Xmera processes (task groups) and tasks are, we can move forward and add modules to a
task. The :ref:`cppModuleTemplate` example provides a basic module, that will be used in this discussion. These
modules demonstrate how to create a prototypical Xmera module.

The modules behave identically and are useful here because they print their module ID when executed. This makes it
easy to observe the module execution order in the simulation.

.. image:: ../../_images/static/qs-bsk-2.svg
   :align: center

Simulation Structure
--------------------

The following simulation creates a single process called ``dynamicsProcess`` and a single 0.2 Hz task called
``dynamicsTask``. As shown above, three instances of :ref:`cppModuleTemplate` are
created, named ``mod1``, ``mod2``, and ``mod3`` respectively. Note that in this simulation we want
modules 2 and 3 to run first, and module 1 to run last.

.. literalinclude:: ../../code-samples/xmera-2.py
   :language: python
   :linenos:
   :lines: 18-

Adding Modules to a Task
-------------------------

To add a module to the simulation, create an instance and give it a unique name using the ``modelTag`` property:

.. code-block:: python

    module = someModule.someModule()
    module.modelTag = "someModuleName"

For C-modules, the class name and instance name are the same. For C++ modules, the first name is the instance, and the
second is the class name (which starts with a capital letter). See the example script for both styles.

Next, add the module to a task using:

.. code-block:: python

    scSim.AddModelToTask("taskName", module, priority)

- The first argument is the task name.
- The second is the module instance.
- The third is an optional priority (integer).

.. warning::

    If the ``priority`` is not provided, it defaults to -1. This means the module will execute after any modules
    with assigned priorities, and in the order in which it was added. This behavior is consistent with how processes
    and tasks are handled.

Module Imports
--------------

In the example Python script:

- The module :ref:`cppModuleTemplate` is imported from ``Xmera.simulation``.

.. note::

    Xmera assigns unique positive ID numbers to modules upon their creation. In the example
    simulation, the three modules receive IDs 1, 2, and 3, corresponding to the order of creation.

Execution Priorities
--------------------

In the script:

- ``Module1`` is added without a priority, so it defaults to -1.
- ``Module2`` and ``Module3`` are given priorities 10 and 5, respectively.

Modules with higher priority values are executed first. This allows for deterministic execution ordering.

.. note::

    A task list can contain C++ and Python-based Xmera modules simultaneously. Python modules are discussed
    in ``pyModules``.

Expected Output
---------------

When you run the script, you should see terminal output similar to:

.. code-block::

    source/code-samples % python3 xmera-2.py
    BSK_INFORMATION: Variable dummy set to 0.000000 in reset.
    BSK_INFORMATION: Variable dummy set to 0.000000 in reset.
    BSK_INFORMATION: Variable dummy set to 0.000000 in reset.
    InitializeSimulation() completed...
    BSK_INFORMATION: C++ Module ID 2 ran Update at 0.000000s
    BSK_INFORMATION: C++ Module ID 3 ran Update at 0.000000s
    BSK_INFORMATION: C++ Module ID 1 ran Update at 0.000000s
    BSK_INFORMATION: C++ Module ID 2 ran Update at 5.000000s
    BSK_INFORMATION: C++ Module ID 3 ran Update at 5.000000s
    BSK_INFORMATION: C++ Module ID 1_Update at 5.000000s

The ``reset()`` method from :ref:`cppModuleTemplate` prints a statement when a variable
is initialized. Since three modules are created, this reset message appears three times—once for each module.

Once ``scSim.InitializeSimulation()`` is called, Xmera begins executing the simulation. The ``Update()`` method
for each module is called in the order determined by their priorities, and the simulation time is printed each time
the module executes. As shown, the desired execution order is achieved based on the set priorities.
