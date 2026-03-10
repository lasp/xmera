# SPDX-License-Identifier: ISC
# Copyright (c) 2021, Autonomous Vehicle System Lab, University of Colorado at Boulder
# Copyright (c) 2025, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder
#

from xmera.utilities import SimulationBaseClass
from xmera.utilities import macros
from xmera.moduleTemplates import cppModuleTemplate
from xmera.architecture import sim_model
from xmera.architecture.messaging import ModuleTemplateMsg, ModuleTemplateMsgPayload

import numpy as np

def test_PySysModel():
    testResults, testMessage = 0, []

    #  Create a sim module as an empty container
    scSim = SimulationBaseClass.SimBaseClass()

    #  create the simulation process
    dynProcess = scSim.CreateNewProcess("dynamicsProcess")

    # create the dynamics task and specify the integration update time
    dynProcess.addTask(scSim.CreateNewTask("dynamicsTask", macros.sec2nano(5.)))

    # create copies of the Xmera modules
    mod1 = cppModuleTemplate.CppModuleTemplate()
    mod1.modelTag = "module1"

    mod2 = cppModuleTemplate.CppModuleTemplate()
    mod2.modelTag = "module2"

    mod3 = cppModuleTemplate.CppModuleTemplate()
    mod3.modelTag = "module3"

    mod4 = PythonModule()
    mod4.modelTag = "pythonModule4"

    mod2.dataInMsg.subscribeTo(mod4.dataOutMsg)

    scSim.AddModelToTask("dynamicsTask", mod1, 0)
    scSim.AddModelToTask("dynamicsTask", mod2, 5)
    scSim.AddModelToTask("dynamicsTask", mod3, 15)
    scSim.AddModelToTask("dynamicsTask", mod4, 10)

    # Set up recording
    mod2MsgRecorder = mod2.dataOutMsg.recorder()
    scSim.AddModelToTask("dynamicsTask", mod2MsgRecorder)

    # initialize Simulation:
    scSim.InitializeSimulation()

    # configure a simulation stop time and execute the simulation run
    scSim.ConfigureStopTime(macros.sec2nano(5.0))
    scSim.ExecuteSimulation()

    if mod4.CallCounts != 2:
        testResults += 1
        testMessage.append("TestPythonModule::updateState was not called")

    if mod2MsgRecorder.dataVector[1,1] == 0:
        testResults += 1
        testMessage.append("Message from TestPythonModule was not connected to message in mod2")
    elif mod2MsgRecorder.dataVector[1,1] == 1:
        testResults += 1
        testMessage.append("TestPythonModule does not run before mod2 despite having greater priority")

    assert testResults < 1, testMessage

class PythonModule(sim_model.SysModel):

    def __init__(self, *args):
        super().__init__(*args)
        self.dataOutMsg = ModuleTemplateMsg()

    def reset(self, currentSimNanos):
        payload = ModuleTemplateMsgPayload()
        payload.dataVector = np.array([0,0,0])
        self.dataOutMsg.write(payload, currentSimNanos, self.moduleID)
        self.bskLogger.bskLog(sim_model.BSK_INFORMATION, "Reset in TestPythonModule")

    def updateState(self, currentSimNanos):
        payload = ModuleTemplateMsgPayload()
        payload.dataVector = self.dataOutMsg.read().dataVector + np.array([0,1,0])
        self.dataOutMsg.write(payload, currentSimNanos, self.moduleID)
        self.bskLogger.bskLog(sim_model.BSK_INFORMATION, f"Python Module ID {self.moduleID} ran Update at {currentSimNanos*1e-9}s")

if __name__ == "__main__":
    test_PySysModel()
