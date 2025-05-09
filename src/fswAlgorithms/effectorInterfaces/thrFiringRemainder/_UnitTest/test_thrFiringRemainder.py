import inspect
import os

import pytest

# Import all of the modules that we are going to be called in this simulation
from xmera.utilities import SimulationBaseClass
from xmera.utilities import unitTestSupport                  # general support file with common unit test functions
from xmera.fswAlgorithms import thrFiringRemainder            # import the module that is to be tested
from xmera.utilities import macros
from xmera.utilities import fswSetupThrusters
from xmera.architecture import messaging

import numpy as np

@pytest.mark.parametrize("resetCheck, dvOn", [
    (False,False),
    (True,False),
    (False,True),
    (True,True)
])

def test_thrFiringRemainder(show_plots, resetCheck, dvOn):

    unitTaskName = "unitTask"               # arbitrary name (don't change)
    unitProcessName = "TestProcess"         # arbitrary name (don't change)

    # Create a sim module as an empty container
    unitTestSim = SimulationBaseClass.SimBaseClass()

    # Create test thread
    fswRate = 0.5
    defaultControlPeriod = 3.0
    testProcessRate = macros.sec2nano(fswRate)  # update process rate update time
    testProc = unitTestSim.CreateNewProcess(unitProcessName)
    testProc.addTask(unitTestSim.CreateNewTask(unitTaskName, testProcessRate))

    # Construct algorithm and associated C++ container
    module = thrFiringRemainder.ThrFiringRemainder()
    module.modelTag = "thrFiringRemainder"

    # Add test module to runtime call list
    unitTestSim.AddModelToTask(unitTaskName, module)

    # Initialize the test module configuration data
    module.thrMinFireTime = 0.2
    module.defaultControlPeriod = defaultControlPeriod
    if dvOn == 1:
        module.baseThrustState = 1
    else:
        module.baseThrustState = 0

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
    maxThrust = 0.5

    for i in range(len(rcsLocationData)):
        fswSetupThrusters.create(rcsLocationData[i], rcsDirectionData[i], maxThrust)
    thrConfigMsg = fswSetupThrusters.writeConfigMessage()
    numThrusters = fswSetupThrusters.getNumOfDevices()

    # setup thruster impulse request message
    thrMessageData = messaging.THRArrayCmdForceMsgPayload()
    if dvOn:
        thrMessageData.thrForce = [-0.5, 0.0, -0.1, -0.2, -0.3, -0.34, -0.39, -0.44]
    else:
        thrMessageData.thrForce = [0.5, 0.05, 0.1, 0.15, 0.19, 0.0, 0.2, 0.49]
    thrForceMsg = messaging.THRArrayCmdForceMsg().write(thrMessageData)

    # Setup logging on the test module output message so that we get all the writes to it
    dataLog = module.onTimeOutMsg.recorder()
    unitTestSim.AddModelToTask(unitTaskName, dataLog)

    # connect messages
    module.thrConfInMsg.subscribeTo(thrConfigMsg)
    module.thrForceInMsg.subscribeTo(thrForceMsg)

    # Need to call the self-init and cross-init methods
    unitTestSim.InitializeSimulation()

    # Set the simulation time.
    # NOTE: the total simulation time may be longer than this value. The
    # simulation is stopped at the next logging event on or after the
    # simulation end time.
    finalTime = 3.0
    unitTestSim.ConfigureStopTime(macros.sec2nano(finalTime))  # seconds to stop simulation
    unitTestSim.ExecuteSimulation()

    if resetCheck:
        # reset the module to test this functionality
        module.reset(macros.sec2nano(finalTime))  # this module reset function needs a time input (in NanoSeconds)

        # run the module again for an additional 2.5 seconds
        unitTestSim.ConfigureStopTime(macros.sec2nano(5.5))  # seconds to stop simulation
        unitTestSim.ExecuteSimulation()

    moduleOutput = dataLog.OnTimeRequest[:, :numThrusters]

    # compute true values
    thrForce = thrMessageData.thrForce
    pulseRemainder = np.zeros([numThrusters])
    trueVector = np.empty([len(moduleOutput), numThrusters])
    idxReset = finalTime / fswRate + 1
    for idx in range(0, len(moduleOutput)):
        onTimes = np.empty([numThrusters])
        # reset at corresponding idx if resetCheck is true,
        # or at idx 0 and 1 as output is the same for time 0 and first time step
        if (resetCheck and idx == idxReset) or idx <= 1:
            controlPeriod = defaultControlPeriod
            pulseRemainder = np.zeros([numThrusters])  # reset pulse remainder
        else:
            controlPeriod = fswRate

        for thrIdx in range(0, numThrusters):
            thrust = thrForce[thrIdx]
            if dvOn:
                thrust += maxThrust
            thrust = max(thrust, 0.0)  # Do not allow thrust requests less than zero
            onTime = thrust / maxThrust * controlPeriod
            onTime += pulseRemainder[thrIdx] * module.thrMinFireTime
            pulseRemainder[thrIdx] = 0.0
            if onTime < module.thrMinFireTime:
                pulseRemainder[thrIdx] = onTime / module.thrMinFireTime
                onTime = 0.0
            elif onTime >= controlPeriod:
                onTime = 1.1 * controlPeriod
            onTimes[thrIdx] = onTime
        trueVector[idx] = onTimes

    # compare the module results to the truth values
    accuracy = 1e-12
    np.testing.assert_allclose(trueVector, moduleOutput, atol=accuracy, verbose=True)


#
# This statement below ensures that the unitTestScript can be run as a
# stand-along python script
#
if __name__ == "__main__":
    test_thrFiringRemainder(
                 True,            # show plots
                 False,           #
                 False
               )
