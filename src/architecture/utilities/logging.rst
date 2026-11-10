Executive Summary
-----------------
Xmera modules write log messages with the `spdlog <https://github.com/gabime/spdlog>`_ library.
Each module gets a logger that has the same name as the ``modelTag`` of the module.
All loggers send their messages to the same sink. The default sink is the terminal.

Each message shows the level and the logger name before the text::

    [error] [mrpPD] mrpPD_C.guidInMsg wasn't connected.

.. list-table:: Log Levels
   :widths: 25 75
   :header-rows: 1

   * - Level
     - Description
   * - ``trace``
     - Information about each small step in the code.
   * - ``debug``
     - Information to find a problem in the code. Remove these messages from completed modules.
   * - ``info``
     - General information about the simulation.
   * - ``warning``
     - A condition that is not usual, but that does not stop the simulation.
   * - ``error``
     - Incorrect behavior that you must correct.
   * - ``critical``
     - A failure that stops the module.
   * - ``off``
     - Use this level only to set a logger. It stops all messages.

The default level is ``debug``. Thus, the loggers write all messages at ``debug`` or a higher level.

Set the Log Level from Python
-----------------------------
For an example, refer to :ref:`scenarioLogging`.

To set the level for all modules, do this step:

#. Use ``set_log_level`` with the name of the level::

    from xmera.architecture import sim_model
    sim_model.set_log_level("warning")

To set the level for one module, do this step:

#. Use ``set_log_level`` with the level and the ``modelTag`` of the module::

    sim_model.set_log_level("error", name=module.modelTag)

A level that you set for one module stays the same when you change the level for all modules.
You can set a level for a module before the module writes its first message.

To remove all levels set by name and set the level of all modules back to ``debug``, use ``reset_log_levels``::

    sim_model.reset_log_levels()

Send the Messages to a Different Sink
-------------------------------------
To send the messages of all loggers to a file, do this step:

#. Use ``set_log_sink`` with the path of the file::

    sim_model.set_log_sink("file:simulation.log")

The other sinks are ``"stdout"``, ``"stderr"`` and ``"null"``. The ``"null"`` sink discards all messages.

Write Log Messages from a C++ Module
------------------------------------
To write a log message from a C++ module, do these steps:

#. Include the logging header in the ``*.cpp`` file of the module:

   .. code-block:: cpp

       #include <architecture/utilities/logging.h>

#. Get the logger of the module and write the message:

   .. code-block:: cpp

       xmera::logging::get(this->modelTag)->warn("Gain K is {:.3f}. The gain must be more than 0.", this->K);

The message uses the format syntax of the ``fmt`` library. The compiler examines the format string
and the arguments. Thus, an incorrect format string causes a compile error.

A logger member is not necessary in a module. A module that does not write messages does not keep a logger.
For a module that writes messages each time that ``updateState`` operates, refer to the Performance section.

A class that is not a ``SysModel`` does not have a ``modelTag``. This type of class uses a logger
name that does not change, for example ``xmera::logging::get("gravity")``.

Write Log Messages from a Python Module
---------------------------------------
To write a log message from a Python module, do this step:

#. Use the ``log`` method of the module::

    self.log("info", f"Reset at {currentSimNanos * 1e-9} s")

The ``log`` method uses the logger that has the ``modelTag`` of the module.
Python code that is not a module can use ``sim_model.log(level, message, name)``.

Performance
-----------
The module does a lookup of its logger each time that it writes a message.
The time for this lookup is small. Thus, the lookup is not a problem for messages in ``reset`` and for errors.

If a module writes messages each time that ``updateState`` operates, keep the logger in a member.
Get the logger in ``reset``:

.. code-block:: cpp

    this->logger = xmera::logging::get(this->modelTag);

To stop the messages of a Monte Carlo worker, set the level to ``off`` or ``error`` in the worker.
