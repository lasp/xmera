# SPDX-License-Identifier: ISC
# Copyright (c) 2021, Autonomous Vehicle System Lab, University of Colorado at Boulder
# Copyright (c) 2025, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder
#

from xmera.moduleTemplates import cppModuleTemplate
from xmera.utilities import SimulationBaseClass
from xmera.utilities import macros


def run():
    """
    Controlling the simulation time
    """

    #  Create a sim module as an empty container
    scSim = SimulationBaseClass.SimBaseClass()

    #  create the simulation process
    dynProcess = scSim.CreateNewProcess("dynamicsProcess")

    # create the dynamics task and specify the integration update time
    dynProcess.addTask("dynamicsTask", macros.sec2nano(1.))

    # create modules
    mod1 = cppModuleTemplate.CppModuleTemplate()
    mod1.modelTag = "module1"
    scSim.AddModelToTask("dynamicsTask", mod1)
    mod1.dummy = -10
    print(mod1.dummy)

    #  initialize Simulation:
    scSim.InitializeSimulation()
    print(mod1.dummy)

    # perform a single Update on all modules
    scSim.TotalSim.singleStepProcesses()
    print(mod1.dummy)

    return


if __name__ == "__main__":
    run()
