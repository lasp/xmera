# SPDX-License-Identifier: ISC
# Copyright (c) 2023, Autonomous Vehicle System Lab, University of Colorado at Boulder
# Copyright (c) 2025, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder
#

from numpy import testing

import xmera.architecture.messaging
from xmera.moduleTemplates import cppModuleTemplate
from xmera.utilities import SimulationBaseClass
from xmera.utilities import macros


def test_msgTimeWritten():
    """
    testing recording timeWritten in a message
    """

    #  Create a sim module as an empty container
    scSim = SimulationBaseClass.SimBaseClass()

    #  create the simulation process
    dynProcess = scSim.CreateNewProcess("dynamicsProcess")

    # create the dynamics task and specify the integration update time
    dynProcess.addTask("dynamicsTask", macros.sec2nano(1.))

    # create modules
    mod1 = cppModuleTemplate.CppModuleTemplate()
    mod1.modelTag = "cModule1"
    scSim.AddModelToTask("dynamicsTask", mod1)
    mod1.dataInMsg.subscribeTo(mod1.dataOutMsg)

    # setup message recording
    msgRec = mod1.dataOutMsg.recorder()
    scSim.AddModelToTask("dynamicsTask", msgRec)

    #  initialize Simulation:
    scSim.InitializeSimulation()

    #   configure a simulation stop time and execute the simulation run
    scSim.ConfigureStopTime(macros.sec2nano(1.0))
    scSim.ExecuteSimulation()

    testing.assert_allclose(msgRec.timesWritten(),
                            msgRec.times(),
                            atol=0.01,
                            err_msg="recorded msg timesWritten was not correct.")


if __name__ == "__main__":
    test_msgTimeWritten()
