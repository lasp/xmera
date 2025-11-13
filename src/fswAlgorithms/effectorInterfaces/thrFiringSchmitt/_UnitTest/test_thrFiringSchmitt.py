import numpy
import pytest
from xmera.architecture import messaging
from xmera.fswAlgorithms import thrFiringSchmitt
from xmera.utilities import SimulationBaseClass
from xmera.utilities import fswSetupThrusters
from xmera.utilities import macros
from xmera.utilities import unitTestSupport


@pytest.mark.parametrize("resetCheck, dvOn", [
    (False, False),
    (True, False),
    (False, True),
    (True, True),
])
def test_thrFiringSchmitt(show_plots, resetCheck, dvOn):
    unitTaskName = "unitTask"               # arbitrary name (don't change)
    unitProcessName = "TestProcess"         # arbitrary name (don't change)

    # Create a sim module as an empty container
    unitTestSim = SimulationBaseClass.SimBaseClass()

    # Create test thread
    testProcessRate = macros.sec2nano(0.5)     # update process rate update time
    testProc = unitTestSim.CreateNewProcess(unitProcessName)
    testProc.addTask(unitTestSim.CreateNewTask(unitTaskName, testProcessRate))

    module = thrFiringSchmitt.ThrFiringSchmitt()
    module.modelTag = "thrFiringSchmitt"

    # Add test module to runtime call list
    unitTestSim.AddModelToTask(unitTaskName, module)

    # Initialize the test module configuration data
    module.setThrMinFireTime(0.2)
    if dvOn == 1:
        module.setBaseThrustState(1)
    else:
        module.setBaseThrustState(0)

    module.setLevelOn(.75)
    module.setLevelOff(.25)

    # setup thruster cluster message
    fswSetupThrusters.clearSetup()
    rcsLocationData = [
        [-0.86360, -0.82550, 1.79070],
        [-0.82550, -0.86360, 1.79070],
        [0.82550, 0.86360, 1.79070],
        [0.86360, 0.82550, 1.79070],
        [-0.86360, -0.82550, -1.79070],
        [-0.82550, -0.86360, -1.79070],
        [0.82550, 0.86360, -1.79070],
        [0.86360, 0.82550, -1.79070]
        ]
    rcsDirectionData = [
        [1.0, 0.0, 0.0],
        [0.0, 1.0, 0.0],
        [0.0, -1.0, 0.0],
        [-1.0, 0.0, 0.0],
        [-1.0, 0.0, 0.0],
        [0.0, -1.0, 0.0],
        [0.0, 1.0, 0.0],
        [1.0, 0.0, 0.0]
        ]

    for i in range(len(rcsLocationData)):
        fswSetupThrusters.create(rcsLocationData[i], rcsDirectionData[i], 0.5)
    thrConfMsg = fswSetupThrusters.writeConfigMessage()
    numThrusters = fswSetupThrusters.getNumOfDevices()
    module.thrConfInMsg.subscribeTo(thrConfMsg)

    # setup thruster impulse request message
    inputMessageData = messaging.THRArrayCmdForceMsgPayload()
    thrCmdMsg = messaging.THRArrayCmdForceMsg()
    module.thrForceInMsg.subscribeTo(thrCmdMsg)

    # Setup logging on the test module output message so that we get all the writes to it
    dataLog = module.onTimeOutMsg.recorder()
    unitTestSim.AddModelToTask(unitTaskName, dataLog)

    # Need to call the self-init and cross-init methods
    unitTestSim.InitializeSimulation()

    # Set the simulation time.
    # NOTE: the total simulation time may be longer than this value. The
    # simulation is stopped at the next logging event on or after the
    # simulation end time.

    if dvOn:
        effReq1 = [0.0, -0.1, -0.2, -0.3, -0.349, -0.351, -0.451, -0.5]
        effReq2 = [0.0, -0.1, -0.2, -0.3, -0.351, -0.351, -0.451, -0.5]
        effReq3 = [0.0, -0.1, -0.2, -0.3, -0.5, -0.351, -0.451, -0.5]
        effReq4 = [0.0, -0.1, -0.2, -0.3, -0.351, -0.351, -0.451, -0.5]

    else:
        effReq1 = [0.5, 0.05, 0.09, 0.11, 0.16, 0.18, 0.2, 0.49]
        effReq2 = [0.5, 0.05, 0.09, 0.11, 0.16, 0.18, 0.2, 0.11]
        effReq3 = [0.5, 0.05, 0.09, 0.11, 0.16, 0.18, 0.2, 0.01]
        effReq4 = [0.5, 0.05, 0.09, 0.11, 0.16, 0.18, 0.2, 0.11]

    inputMessageData.thrForce = effReq1
    thrCmdMsg.write(inputMessageData)
    unitTestSim.ConfigureStopTime(macros.sec2nano(1.0))        # seconds to stop simulation
    unitTestSim.ExecuteSimulation()

    inputMessageData.thrForce = effReq2
    thrCmdMsg.write(inputMessageData)
    unitTestSim.ConfigureStopTime(macros.sec2nano(2.0))        # seconds to stop simulation
    unitTestSim.ExecuteSimulation()

    inputMessageData.thrForce = effReq3
    thrCmdMsg.write(inputMessageData)
    unitTestSim.ConfigureStopTime(macros.sec2nano(2.5))        # seconds to stop simulation
    unitTestSim.ExecuteSimulation()

    inputMessageData.thrForce = effReq4
    thrCmdMsg.write(inputMessageData)
    unitTestSim.ConfigureStopTime(macros.sec2nano(3.0))        # seconds to stop simulation
    unitTestSim.ExecuteSimulation()

    if resetCheck:
        # reset the module to test this functionality
        module.reset(macros.sec2nano(3.0))     # this module reset function needs a time input (in NanoSeconds)

        # run the module again for an additional 1.0 seconds
        unitTestSim.ConfigureStopTime(macros.sec2nano(5.5))        # seconds to stop simulation
        unitTestSim.ExecuteSimulation()

    # This pulls the actual data log from the simulation run.
    moduleOutput = dataLog.OnTimeRequest[:, :numThrusters]

    # set the filtered output truth states
    if resetCheck==1:
        if dvOn == 1:
            trueVector = [
                   [2.0, 2.0, 2.0, 2.0, 2.0, 2.0, 2.0, 2.0],
                   [2.0, 2.0, 2.0, 2.0, 2.0, 2.0, 2.0, 2.0],
                   [0.55, 0.4, 0.3, 0.2, 0.2, 0.0, 0.0, 0.0],
                   [0.55, 0.4, 0.3, 0.2, 0.2, 0.0, 0.0, 0.0],
                   [0.55, 0.4, 0.3, 0.2, 0.2, 0.0, 0.0, 0.0],
                   [0.55, 0.4, 0.3, 0.2, 0.0, 0.0, 0.0, 0.0],
                   [0.55, 0.4, 0.3, 0.2, 0.0, 0.0, 0.0, 0.0],
                   [2.0, 2.0, 2.0, 2.0, 2.0, 2.0, 2.0, 2.0],
                   [0.55, 0.4, 0.3, 0.2, 0.0, 0.0, 0.0, 0.0],
                   [0.55, 0.4, 0.3, 0.2, 0.0, 0.0, 0.0, 0.0],
                   [0.55, 0.4, 0.3, 0.2, 0.0, 0.0, 0.0, 0.0],
                   [0.55, 0.4, 0.3, 0.2, 0.0, 0.0, 0.0, 0.0],
                   ]
        else:
            trueVector = [
                   [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                   [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                   [0.55, 0.0, 0.0, 0.0, 0.2, 0.2, 0.2, 0.49],
                   [0.55, 0.0, 0.0, 0.0, 0.2, 0.2, 0.2, 0.2],
                   [0.55, 0.0, 0.0, 0.0, 0.2, 0.2, 0.2, 0.2],
                   [0.55, 0.0, 0.0, 0.0, 0.2, 0.2, 0.2, 0.0],
                   [0.55, 0.0, 0.0, 0.0, 0.2, 0.2, 0.2, 0.0],
                   [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                   [0.55, 0.0, 0.0, 0.0, 0.2, 0.2, 0.2, 0.0],
                   [0.55, 0.0, 0.0, 0.0, 0.2, 0.2, 0.2, 0.0],
                   [0.55, 0.0, 0.0, 0.0, 0.2, 0.2, 0.2, 0.0],
                   [0.55, 0.0, 0.0, 0.0, 0.2, 0.2, 0.2, 0.0],
                   ]

    else:
        if dvOn == 1:
            trueVector = [
                   [2.0, 2.0, 2.0, 2.0, 2.0, 2.0, 2.0, 2.0],
                   [2.0, 2.0, 2.0, 2.0, 2.0, 2.0, 2.0, 2.0],
                   [0.55, 0.4, 0.3, 0.2, 0.2, 0.0, 0.0, 0.0],
                   [0.55, 0.4, 0.3, 0.2, 0.2, 0.0, 0.0, 0.0],
                   [0.55, 0.4, 0.3, 0.2, 0.2, 0.0, 0.0, 0.0],
                   [0.55, 0.4, 0.3, 0.2, 0.0, 0.0, 0.0, 0.0],
                   [0.55, 0.4, 0.3, 0.2, 0.0, 0.0, 0.0, 0.0],
                   ]
        else:
            trueVector = [
                   [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                   [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                   [0.55, 0.0, 0.0, 0.0, 0.2, 0.2, 0.2, 0.49],
                   [0.55, 0.0, 0.0, 0.0, 0.2, 0.2, 0.2, 0.2],
                   [0.55, 0.0, 0.0, 0.0, 0.2, 0.2, 0.2, 0.2],
                   [0.55, 0.0, 0.0, 0.0, 0.2, 0.2, 0.2, 0.0],
                   [0.55, 0.0, 0.0, 0.0, 0.2, 0.2, 0.2, 0.0],
                   ]

    numpy.testing.assert_allclose(moduleOutput, trueVector, atol=1e-12, err_msg="OnTimeRequest")


#
# This statement below ensures that the unitTestScript can be run as a
# stand-along python script
#
if __name__ == "__main__":
    test_thrFiringSchmitt(False,True,False)
