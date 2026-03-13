.. _xmeraPrinciples-8:

Advanced: Enabling and Disabling Tasks
======================================

.. sidebar:: Source Code

    The python code shown below can be downloaded :download:`here </../../docs/source/code-samples/xmera-8.py>`.

This section demonstrates how Xmera tasks can be enabled or disabled during runtime. Why is this useful? You might
set up one group of modules to perform sun-pointing behavior, and another group for a science-pointing mode. By enabling
or disabling tasks dynamically, you can define all tasks at the start of the simulation and selectively control which
flight software modules are actively executed.

.. image:: ../../_images/static/qs-bsk-8.svg
   :align: center

The sample script sets up a single process containing two tasks, named ``task1`` and ``task2``. Modules are assigned to
each of these tasks.

.. literalinclude:: ../../code-samples/xmera-8.py
   :language: python
   :linenos:
   :lines: 18-

After the typical module initialization, the script executes a single simulation step. The terminal output confirms that
both tasks are enabled, as both sets of modules are executed.

To disable **all** tasks within a process, use the ``disableTasks()`` method on the process variable. The script executes
another simulation step and prints output before and after to confirm that no tasks are running when they are disabled.

To re-enable a specific task, use the ``SimulationBaseClass`` method ``enableTask(name)``. The argument is the name of
the task you wish to enable. After another simulation step, the output confirms that the enabled task's modules are once
again executed.

Similarly, to disable a specific task, use the method ``disableTask(name)``. The name of the task must be passed as a
string.

The expected output from executing the script looks as follows:

.. code-block::

    (.venv) source/code-samples % python xmera-8.py
    BSK_INFORMATION: Variable dummy set to 0.000000 in reset.
    BSK_INFORMATION: Variable dummy set to 0.000000 in reset.
    BSK_INFORMATION: Module ID 1 ran Update at 0.000000s
    BSK_INFORMATION: Module ID 2 ran Update at 0.000000s
    all tasks disabled
    BSK executed a single simulation step
    BSK_INFORMATION: Module ID 1 ran Update at 2.000000s
    BSK executed a single simulation step
    BSK_INFORMATION: Module ID 1 ran Update at 3.000000s
    BSK_INFORMATION: Module ID 2 ran Update at 3.000000s
    BSK executed a single simulation step
    BSK_INFORMATION: Module ID 1 ran Update at 4.000000s
    BSK executed a single simulation step
