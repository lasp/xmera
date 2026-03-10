.. _xmeraPrinciples-4:

Recording Messages
==================

.. sidebar:: Source Code

    The Python code shown below can be downloaded :download:`here </../../docs/source/code-samples/xmera-4.py>`.

Once a simulation is functioning with connected messages between modules, you might want to access the resulting data.
This is done by creating message recorder objects that store a time history of the message data.

.. image:: ../../_images/static/qs-bsk-4.svg
   :align: center

The figure above illustrates a sample simulation setup. The single test module `mod1` has its output message connected
to its input message, forming a feedback loop that causes the output to change over time. See the module code for
details on the simple mathematical operations being performed. To track the state of the message at various time steps,
message recorders are added to the simulation.

The corresponding simulation script is included below. Since the recorded data will be plotted, the `matplotlib`
library is imported, along with the `unitTestSupport` helper module from `Xmera.utilities`.

.. literalinclude:: ../../code-samples/xmera-4.py
   :language: python
   :linenos:
   :lines: 18-

Adding a Message Recorder
-------------------------

After creating a module instance and adding it to a task list, you can set up a message recorder. The syntax is as
follows. Suppose you want to record `module.someOutMsg`. This message can be an output or input message. The recorder
object is created using:

::

    someMsgRec = module.someOutMsg.recorder()
    scSim.AddModelToTask("taskName", someMsgRec)

The first line sets up the recorder to monitor `someOutMsg`. As with any simulation component, it must be added to a
task to execute during simulation updates. By default, the recorder captures data at the same rate as the task's update
frequency. To reduce how often data points are recorded, provide a `minUpdateTime` argument:

::

    someMsgRec = module.someOutMsg.recorder(minUpdateTime)

Here, `minUpdateTime` is the minimum interval (in nanoseconds) between recordings.

In the script example above, the recorder `msgRec` records at the task's update rate. Meanwhile, `msgRec20` records
only once every 20 seconds, demonstrating how to manage data volume efficiently.

Pulling the Recorded Message Data
---------------------------------

Once the simulation completes, the data is stored in the respective recorder objects (`msgRec`, `msgRec20`, etc.). To
access recorded variables, use `msgRec.variable`, where `variable` is a specific field in the message structure.

Use `msgRec.times()` to retrieve the array of time stamps when data was recorded. The method `msgRec.timesWritten()`
returns time stamps of when the message was written. Why two time arrays? Imagine an output message updated every 3
seconds, while the recorder checks it every second. The `timesWritten()` array will contain repeated values until the
message updates again.

In the referenced simulation, the output message contains only the array `dataVector`. It is recorded at a 1 Hz rate
by `msgRec` and at 20-second intervals by `msgRec20`. The simulation produces the following plot:

.. image:: /_images/Scenarios/bsk-4.svg
   :align: center

Clearing the Message Recorder Data Log
--------------------------------------

Recorders will continue appending data as long as the simulation runs. If you pause, read data, and then resume, the
recording continues from where it left off. To clear the existing data and start fresh, use the `.clear()` method:

::

    scRec.clear()

This ensures only new data is recorded after the method is called.

Reading the Current Value of a Message
--------------------------------------

To access the most recent data in a message, use the `read` method:

::

    msgCopy = msg.read()

This returns a snapshot of the current message content.
