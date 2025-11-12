#
#   Unit Test Script
#   Module Name:        thrMomentumDumping
#   Author:             Hanspeter Schaub
#   Creation Date:      August 21, 2016
#

import inspect
import os
import numpy as np

import pytest

filename = inspect.getframeinfo(inspect.currentframe()).filename
path = os.path.dirname(os.path.abspath(filename))

# Import all of the modules that we are going to be called in this simulation
from xmera.utilities import SimulationBaseClass
from xmera.utilities import unitTestSupport                  # general support file with common unit test functions
from xmera.fswAlgorithms import thrMomentumDumping            # import the module that is to be tested
from xmera.utilities import macros
from xmera.utilities import fswSetupThrusters
from xmera.architecture import messaging

# Uncomment this line is this test is to be skipped in the global unit test run, adjust message as needed.
# @pytest.mark.skipif(conditionstring)
# Uncomment this line if this test has an expected failure, adjust message as needed.
# @pytest.mark.xfail(conditionstring)
# Provide a unique test method name, starting with 'test_'.
# The following 'parametrize' function decorator provides the parameters and expected results for each
#   of the multiple test runs for this test.
@pytest.mark.parametrize("resetCheck, largeMinFireTime, maxNumOfDtFiringTimes", [
    (False, False, False)
    ,(True, False, False)
    ,(False, True, False)
    ,(False, False, True)
])

# update "module" in this function name to reflect the module name
def test_thrMomentumDumping(show_plots, resetCheck, largeMinFireTime, maxNumOfDtFiringTimes):
    """Module Unit Test"""
    # each test method requires a single assert method to be called
    unitTaskName = "unitTask"               # arbitrary name (don't change)
    unitProcessName = "TestProcess"         # arbitrary name (don't change)

    # Create a sim module as an empty container
    unitTestSim = SimulationBaseClass.SimBaseClass()

    # Create test thread
    if maxNumOfDtFiringTimes:
        testProcessRate = macros.sec2nano(0.1)     # update process rate update times
        simTime = 2.0
    else:
        testProcessRate = macros.sec2nano(0.5)     # update process rate update time
        simTime = 3.0
    testProc = unitTestSim.CreateNewProcess(unitProcessName)
    testProc.addTask(unitTestSim.CreateNewTask(unitTaskName, testProcessRate))


    # Construct algorithm and associated C++ container
    module = thrMomentumDumping.ThrMomentumDumping()
    module.modelTag = "thrMomentumDumping"

    # Add test module to runtime call list
    unitTestSim.AddModelToTask(unitTaskName, module)

    # Initialize the test module configuration data
    if maxNumOfDtFiringTimes:
        module.maxCounterValue = 5
        module.maxNumOfDtFiringTimes = 3
    else:
        module.maxCounterValue = 2

    if largeMinFireTime:
        module.thrMinFireTime = 0.200         # seconds
    else:
        module.thrMinFireTime = 0.020         # seconds

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
        fswSetupThrusters.create(rcsLocationData[i], rcsDirectionData[i], 2.0)
    thrConfInMsg = fswSetupThrusters.writeConfigMessage()
    numThrusters = fswSetupThrusters.getNumOfDevices()

    # setup thruster impulse request message
    DeltaPInMsgData = messaging.THRArrayCmdForceMsgPayload()
    DeltaPInMsgData.thrForce = [1.2, 0.2, 0.0, 1.6, 1.2, 0.2, 1.6, 0.0]
    deltaPInMsg = messaging.THRArrayCmdForceMsg().write(DeltaPInMsgData)

    # setup the commanded angular momentum change message
    DeltaHInMsgData = messaging.CmdTorqueBodyMsgPayload()
    DeltaHInMsgData.torqueRequestBody = [0., 0., 0.]
    deltaHInMsg = messaging.CmdTorqueBodyMsg()

    # Setup logging on the test module output message so that we get all the writes to it
    dataLog = module.thrusterOnTimeOutMsg.recorder()
    unitTestSim.AddModelToTask(unitTaskName, dataLog)

    # connect messages
    module.thrusterImpulseInMsg.subscribeTo(deltaPInMsg)
    module.thrusterConfInMsg.subscribeTo(thrConfInMsg)
    module.deltaHInMsg.subscribeTo(deltaHInMsg)

    # Need to call the self-init and cross-init methods
    unitTestSim.InitializeSimulation()

    # Set the simulation time.
    # NOTE: the total simulation time may be longer than this value. The
    # simulation is stopped at the next logging event on or after the
    # simulation end time.
    unitTestSim.ConfigureStopTime(macros.sec2nano(0.5))        # seconds to stop simulation
    unitTestSim.ExecuteSimulation()

    # write the input Delta_H message
    deltaHInMsg.write(DeltaHInMsgData, macros.sec2nano(0.5))

    unitTestSim.ConfigureStopTime(macros.sec2nano(simTime))        # seconds to stop simulation

    # Begin the simulation time run set above
    unitTestSim.ExecuteSimulation()

    if resetCheck:
        # reset the module to test this functionality
        module.reset(macros.sec2nano(3.0))     # this module reset function needs a time input (in NanoSeconds)

        # run the module again for an additional 1.0 seconds
        unitTestSim.ConfigureStopTime(macros.sec2nano(3.5))        # seconds to stop simulation
        unitTestSim.ExecuteSimulation()

        # re-write the input Delta_H message so that it checks for a new message
        deltaHInMsg.write(DeltaHInMsgData, macros.sec2nano(3.5))

        unitTestSim.ConfigureStopTime(macros.sec2nano(5.5))        # seconds to stop simulation
        unitTestSim.ExecuteSimulation()

    # This pulls the actual data log from the simulation run.
    moduleOutput = dataLog.OnTimeRequest[:, :numThrusters]
    # set the filtered output truth states
    if resetCheck==1:
        trueVector = [
                   [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                   [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                   [0.5, 0.1, 0.0, 0.5, 0.5, 0.1, 0.5, 0.0],
                   [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                   [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                   [0.1, 0.0, 0.0, 0.3, 0.1, 0.0, 0.3, 0.0],
                   [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                   [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                   [0.5, 0.1, 0.0, 0.5, 0.5, 0.1, 0.5, 0.0],
                   [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                   [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                   [0.1, 0.0, 0.0, 0.3, 0.1, 0.0, 0.3, 0.0]
                   ]
    else:
        if largeMinFireTime:
            trueVector = [
                [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                [0.5, 0.0, 0.0, 0.5, 0.5, 0.0, 0.5, 0.0],
                [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                [0.0, 0.0, 0.0, 0.3, 0.0, 0.0, 0.3, 0.0],
                [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
            ]
        elif not largeMinFireTime and not maxNumOfDtFiringTimes:
            trueVector = [
                       [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                       [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                       [0.5, 0.1, 0.0, 0.5, 0.5, 0.1, 0.5, 0.0],
                       [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                       [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                       [0.1, 0.0, 0.0, 0.3, 0.1, 0.0, 0.3, 0.0],
                       [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
                       ]
        else:
            trueVector = [
                [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                [0.3, 0.1, 0.0, 0.3, 0.3, 0.1, 0.3, 0.0],
                [0.3, 0.0, 0.0, 0.3, 0.3, 0.0, 0.3, 0.0],
                [0.3, 0.0, 0.0, 0.3, 0.3, 0.0, 0.3, 0.0],
                [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                [0.3, 0.0, 0.0, 0.3, 0.3, 0.0, 0.3, 0.0],
                [0.2, 0.0, 0.0, 0.3, 0.2, 0.0, 0.3, 0.0],
                [0.1, 0.0, 0.0, 0.3, 0.1, 0.0, 0.3, 0.0],
                [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                [0.0, 0.0, 0.0, 0.2, 0.0, 0.0, 0.2, 0.0],
                [0.0, 0.0, 0.0, 0.1, 0.0, 0.0, 0.1, 0.0],
                [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
            ]

    # compare the module results to the truth values
    np.testing.assert_allclose(moduleOutput, trueVector, rtol=0, atol=1e-12, verbose=True)

#
# This statement below ensures that the unitTestScript can be run as a
# stand-along python script
#
if __name__ == "__main__":
    test_thrMomentumDumping(              # update "module" in function name
                 True,
                 False,             # resetCheck
                 False,              # largeMinFireTime
        True
               )
