.. _xmeraPrinciples-6:

Setting and Recording Module Variables
======================================

.. sidebar:: Source Code

    The Python code shown below can be downloaded :download:`here </../../docs/source/code-samples/xmera-6.py>`.

In some cases, it’s useful to record internal variables from a module, not just the input or output messages. This can
be done using Python, though note that recording module variables through Python can slow down the simulation since
the logging happens at the scripting layer.

The simulation structure is shown in the figure below. Two modules are created and added to the same task. There are
no message connections in this example. Instead, the focus is on recording internal module variables directly.

.. image:: ../../_images/static/qs-bsk-6.svg
   :align: center

The code snippet below sets up the modules as in previous examples. The internal variable `someVariable` from
`someModule` is set in Python using:

::

    someModule.someVariable = ...

.. literalinclude:: ../../code-samples/xmera-6.py
   :language: python
   :linenos:
   :lines: 18-

Logging a Module Variable
--------------------------

To record a module variable, use the following syntax:

::

    moduleLogger = module.logger(variableName, recordingTime)

Here, `variableName` can be a string (for one variable) or a list of strings (to log multiple variables). This tells
the logger which internal variables to track.

The `recordingTime` argument is optional. It defines the minimum interval (in nanoseconds) between data recordings.
If this argument is not provided, the logging occurs at the module's update rate.

After the simulation runs, the data can be accessed using:

::

    moduleLogger.variableName

This returns the values of the specified variable over time. To retrieve the times at which the data was recorded, use:

::

    moduleLogger.times()

These time values are in nanoseconds.

Example Output
--------------

Running the example script should produce output similar to the following:

.. code-block::

    source/code-samples % python xmera-6.py
    BSK_INFORMATION: Variable dummy set to 0.000000 in reset.
    BSK_INFORMATION: Variable dummy set to 0.000000 in reset.
    BSK_INFORMATION: Module ID 1 ran Update at 0.000000s
    BSK_INFORMATION: Module ID 2 ran Update at 0.000000s
    BSK_INFORMATION: Module ID 1 ran Update at 1.000000s
    BSK_INFORMATION: Module ID 2 ran Update at 1.000000s
    Times:  [         0 1000000000]
    mod1.dummy:
    [1. 2.]
    mod2.dummy:
    [1. 2.]
    mod2.dumVector:
    [[1. 2. 3.]
     [1. 2. 3.]]

Clearing the Data Log
----------------------

Like message recorders, loggers accumulate data over the simulation runtime. If you stop the simulation, access the
data, and then resume, the data collection continues from where it left off. To reset the log and capture only new
data, use the `.clear()` method:

::

    moduleLogger.clear()

Advanced Data Logging
----------------------

The `module.logger(variableName)` method provides a simple way to log variables. However, for more advanced needs—such
as logging derived values or data from multiple sources—you can use:

::

    Xmera.utilities.pythonVariableLogger

This class allows for flexible logging setups with pre-processing logic before storing the data. See
:ref:`scenarioFuelSlosh` for an example of advanced logger usage.
