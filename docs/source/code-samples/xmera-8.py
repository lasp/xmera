# SPDX-License-Identifier: ISC
# Copyright (c) 2021, Autonomous Vehicle System Lab, University of Colorado at Boulder
# Copyright (c) 2025, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder
#

from xmera.moduleTemplates import cppModuleTemplate
from xmera.utilities import SimulationBaseClass
from xmera.utilities import macros


def run():
    """
    Illustration of enabling and disabling tasks
    """

    #  Create a sim module as an empty container
    scSim = SimulationBaseClass.SimBaseClass()

    #  create the simulation process
    dynProcess = scSim.CreateNewProcess("dynamicsProcess")

    # create the dynamics task and specify the integration update time
    dynProcess.addTask("task1", macros.sec2nano(1.))
    dynProcess.addTask("task2", macros.sec2nano(1.))

    # create modules
    mod2 = cppModuleTemplate.CppModuleTemplate()
    mod2.modelTag = "module2"
    scSim.AddModelToTask("task2", mod2)

    #  initialize Simulation:
    scSim.InitializeSimulation()

    # execute BSK for a single step
    scSim.singleStepProcesses()

    dynProcess.disableTasks()
    print("all tasks disabled")
    scSim.singleStepProcesses()
    print("BSK executed a single simulation step")

    scSim.enableTask("task2")
    scSim.singleStepProcesses()
    print("BSK executed a single simulation step")

    scSim.disableTask("task2")
    scSim.singleStepProcesses()
    print("BSK executed a single simulation step")

    return


if __name__ == "__main__":
    run()
