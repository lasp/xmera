# SPDX-License-Identifier: ISC
# Copyright (c) 2021, Autonomous Vehicle System Lab, University of Colorado at Boulder
# Copyright (c) 2025, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder
#

from xmera.moduleTemplates import cppModuleTemplate
from xmera.utilities import SimulationBaseClass
from xmera.utilities import macros


def run():
    """
    Illustration of adding xmera modules to a task
    """

    #  Create a sim module as an empty container
    scSim = SimulationBaseClass.SimBaseClass()

    #  create the simulation process
    dynProcess = scSim.CreateNewProcess("dynamicsProcess")

    # create the dynamics task and specify the integration update time
    dynProcess.addTask("dynamicsTask", macros.sec2nano(5.))

    # create copies of the xmera modules
    mod1 = cppModuleTemplate.CppModuleTemplate()
    mod1.modelTag = "module1"

    mod2 = cppModuleTemplate.CppModuleTemplate()
    mod2.modelTag = "module2"

    mod3 = cppModuleTemplate.CppModuleTemplate()
    mod3.modelTag = "module3"

    scSim.AddModelToTask("dynamicsTask", mod1)
    scSim.AddModelToTask("dynamicsTask", mod2, 10)
    scSim.AddModelToTask("dynamicsTask", mod3, 5)

    #  initialize Simulation:
    scSim.InitializeSimulation()
    print("InitializeSimulation() completed...")

    #   configure a simulation stop time and execute the simulation run
    scSim.ConfigureStopTime(macros.sec2nano(5.0))
    scSim.ExecuteSimulation()

    return


if __name__ == "__main__":
    run()
