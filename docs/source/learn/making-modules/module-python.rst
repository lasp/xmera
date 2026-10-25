.. _module-python:

Modules in Python
=================

.. sidebar:: Source Code

    The Python code shown below can be downloaded :download:`here </../../docs/source/code-samples/making-pyModules.py>`.

Python modules are a good alternative to native C++ modules for quick prototyping.
They are defined entirely in a Python script, which means that there is no need
for a header (``.h``), definition (``.cpp``), or SWIG interface file (``.i``). However, they
are much slower than native modules, which will significantly slow down your simulation.

Python modules are implemented by subclassing ``SysModel`` from ``Xmera.architecture.sysModel``.
Then, one can implement the ``__init__``,
``reset``, and ``updateState`` methods in the same way that one would
implement these methods in C++. Remember to always call ``__init__`` of
the parent class ``SysModel`` if you are implementing your own ``__init__``.

The ``modelTag`` value of these python BSK modules will be a unique positive number like any other BSK module.

To write a log message, a Python module uses its ``log`` method, for example ``self.log("info", "Reset")``.
The message uses the logger that has the ``modelTag`` of the module. Additionally,
you may declare any other variables, methods, messages, etc. within your Python module.

The script below expands on the code shown in :ref:`xmeraPrinciples-2` to include
a Python module.

.. literalinclude:: ../../code-samples/making-pyModules.py
   :language: python
   :linenos:
   :lines: 18-

Running the above code prints:

.. code-block::

    (.venv) source/code-samples % python making-pyModules.py
    [info] [module3] Variable dummy set to 0.000000 in reset.
    [info] [pythonModule4] Reset in TestPythonModule
    [info] [module2] Variable dummy set to 0.000000 in reset.
    [info] [module1] Variable dummy set to 0.000000 in reset.
    InitializeSimulation() completed...
    [info] [module3] Module ID 3 ran Update at 0.000000s
    [info] [pythonModule4] Python Module ID 4 ran Update at 0.0s
    [info] [module2] Module ID 2 ran Update at 0.000000s
    [info] [module1] Module ID 1 ran Update at 0.000000s
    [info] [module3] Module ID 3 ran Update at 5.000000s
    [info] [pythonModule4] Python Module ID 4 ran Update at 5.0s
    [info] [module2] Module ID 2 ran Update at 5.000000s
    [info] [module1] Module ID 1 ran Update at 5.000000s
    Recorded mod2.dataOutMsg.dataVector:  [[2. 1. 0.]
     [5. 2. 0.]]

Note how the Python module made use of ``self.log``, the ``reset``
and ``updateState`` were called, how the priority of the Python
module was respected, and how messaging happened between a C++
and Python module.

The scenario :ref:`scenarioAttitudePointingPy` further shows how to define Python modules.
