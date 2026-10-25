# SPDX-License-Identifier: ISC
# Copyright (c) 2016, Autonomous Vehicle System Lab, University of Colorado at Boulder
# Copyright (c) 2026, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder
#

r"""
Overview
--------

This scenario shows how to set the log level of Xmera modules from Python.
The simulation has two template modules. Each module writes ``info`` messages in ``reset`` and in ``updateState``.

The scenario has three cases:

- Case 0 uses the default level ``debug``. The messages of the two modules show.
- Case 1 sets the level of all modules to ``warning``. No ``info`` messages show.
- Case 2 sets the level of only the first module to ``error``. Only the messages of the second module show.

For more information about logging, refer to the ``logging.rst`` file in ``src/architecture/utilities``.

"""

import inspect
import os

filename = inspect.getframeinfo(inspect.currentframe()).filename
path = os.path.dirname(os.path.abspath(filename))

from xmera.utilities import SimulationBaseClass
from xmera.moduleTemplates import cppModuleTemplate
from xmera.utilities import macros
from xmera.architecture import sim_model
from xmera.architecture import messaging


def run(case, logFile=""):
    """
    At the end of the python script you can specify the following example parameters.

    Args:
        case (int):

            ======  ==================================================
             Int    Definition
            ======  ==================================================
              0     Uses the default log level ``debug``
              1     Sets the log level of all modules to ``warning``
              2     Sets the log level of the first module to ``error``
            ======  ==================================================

        logFile (str): If not empty, the loggers write the messages to this file
            instead of to the terminal.
    """
    if logFile:
        sim_model.set_log_sink("file:" + logFile)

    if case == 1:
        sim_model.set_log_level("warning")

    unitTaskName = "unitTask"
    unitProcessName = "TestProcess"

    unitTestSim = SimulationBaseClass.SimBaseClass()

    testProcessRate = macros.sec2nano(0.5)
    testProc = unitTestSim.CreateNewProcess(unitProcessName)
    testProc.addTask(unitTestSim.CreateNewTask(unitTaskName, testProcessRate))

    inputMessageData = messaging.ModuleTemplateMsgPayload()
    inputMessageData.dataVector = [1.0, 1.0, 0.7]
    dataMsg = messaging.ModuleTemplateMsg().write(inputMessageData)

    modules = []
    for tag in ("cppModuleTemplate1", "cppModuleTemplate2"):
        module = cppModuleTemplate.CppModuleTemplate()
        module.modelTag = tag
        module.dataInMsg.subscribeTo(dataMsg)
        module.dummy = 1
        unitTestSim.AddModelToTask(unitTaskName, module)
        modules.append(module)

    if case == 2:
        # The level of one module can change at any time, also after the module is made
        sim_model.set_log_level("error", name=modules[0].modelTag)

    unitTestSim.InitializeSimulation()
    unitTestSim.ConfigureStopTime(macros.sec2nano(1.0))
    unitTestSim.ExecuteSimulation()


if __name__ == "__main__":
    run(
        2,  # case 0 uses the default level, case 1 sets all modules, case 2 sets one module
    )
