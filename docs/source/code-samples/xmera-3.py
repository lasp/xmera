# SPDX-License-Identifier: ISC
# Copyright (c) 2021, Autonomous Vehicle System Lab, University of Colorado at Boulder
# Copyright (c) 2025, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder
#

from xmera.moduleTemplates import cppModuleTemplate
from xmera.utilities import SimulationBaseClass
from xmera.utilities import macros


def run():
    """
    Illustration of connecting module messages
    """

    #  Create a sim module as an empty container
    scSim = SimulationBaseClass.SimBaseClass()

    #  create the simulation process
    dynProcess = scSim.CreateNewProcess("dynamicsProcess")

    # create the dynamics task and specify the integration update time
    dynProcess.addTask(scSim.CreateNewTask("dynamicsTask", macros.sec2nano(5.)))

    # create modules
    mod1 = cppModuleTemplate.CppModuleTemplate()
    mod1.modelTag = "module1"

    mod2 = cppModuleTemplate.CppModuleTemplate()
    mod2.modelTag = "module2"

    # add modules to task list
    scSim.AddModelToTask("dynamicsTask", mod1)
    scSim.AddModelToTask("dynamicsTask", mod2)

    # connect messages
    mod2.dataInMsg.subscribeTo(mod1.dataOutMsg)
    mod1.dataInMsg.subscribeTo(mod2.dataOutMsg)

    #  initialize Simulation:
    scSim.InitializeSimulation()

    #   configure a simulation stop time and execute the simulation run
    scSim.ConfigureStopTime(macros.sec2nano(5.0))
    scSim.ExecuteSimulation()

    return


if __name__ == "__main__":
    run()
