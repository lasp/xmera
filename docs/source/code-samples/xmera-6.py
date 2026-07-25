# SPDX-License-Identifier: ISC
# Copyright (c) 2021, Autonomous Vehicle System Lab, University of Colorado at Boulder
# Copyright (c) 2025, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder
#

from xmera.moduleTemplates import cppModuleTemplate
from xmera.utilities import SimulationBaseClass
from xmera.utilities import macros


def run():
    """
    Illustration of setting and recording module variables
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

    mod2 = cppModuleTemplate.CppModuleTemplate()
    mod2.modelTag = "module2"
    scSim.AddModelToTask("dynamicsTask", mod2)

    # set module variables
    mod1.dummy = 1
    mod1.dumVector = [1., 2., 3.]
    mod2.dummy = 1
    mod2.dumVector = [1., 2., 3.]

    # request these module variables to be recorded
    mod1Logger = mod1.logger("dummy", macros.sec2nano(1.))
    scSim.AddModelToTask("dynamicsTask", mod1Logger)
    mod2WrapLogger = mod2.logger(["dummy", "dumVector"], macros.sec2nano(1.))
    scSim.AddModelToTask("dynamicsTask", mod2WrapLogger)

    #  initialize Simulation:
    scSim.InitializeSimulation()

    #   configure a simulation stop time and execute the simulation run
    scSim.ConfigureStopTime(macros.sec2nano(1.0))
    scSim.ExecuteSimulation()

    # Print when were the variables logged (the same for every logged variable)
    print("Times: ", mod1Logger.times())

    # Print values logged
    print("mod1.dummy:")
    print(mod1Logger.dummy)
    print("mod2.dummy:")
    print(mod2WrapLogger.dummy)
    print("mod2.dumVector:")
    print(mod2WrapLogger.dumVector)

if __name__ == "__main__":
    run()
