# SPDX-License-Identifier: ISC
# Copyright (c) 2016, Autonomous Vehicle System Lab, University of Colorado at Boulder
#

import pytest

from xmera.utilities import SimulationBaseClass
from xmera.fswAlgorithms import thrFiringRemainder_C
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
def test_thrFiringRemainderC(show_plots, resetCheck, dvOn):
    unitTaskName = "unitTask"
    unitProcessName = "TestProcess"
    unitTestSim = SimulationBaseClass.SimBaseClass()

    fswRate = 0.5
    defaultControlPeriod = 3.0
    testProcessRate = macros.sec2nano(fswRate)  # update process rate update time
    testProc = unitTestSim.CreateNewProcess(unitProcessName)
    testProc.addTask(unitTaskName, testProcessRate)

    module = thrFiringRemainder_C.ThrFiringRemainder_C()
    module.modelTag = "thrFiringRemainder_C"

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

    dataLog = module.onTimeOutMsg.recorder()
    unitTestSim.AddModelToTask(unitTaskName, dataLog)

    # connect messages
    module.thrConfInMsg.subscribeTo(thrConfigMsg)
    module.thrForceInMsg.subscribeTo(thrForceMsg)

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
    pulseRemainder = np.zeros(numThrusters)
    trueVector = np.empty([len(moduleOutput), numThrusters])
    idxReset = finalTime / fswRate + 1
    for idx in range(len(moduleOutput)):
        onTimes = np.empty(numThrusters)
        # reset at corresponding idx if resetCheck is true,
        # or at idx 0 and 1 as output is the same for time 0 and first time step
        if (resetCheck and idx == idxReset) or idx <= 1:
            controlPeriod = defaultControlPeriod
            pulseRemainder = np.zeros(numThrusters)  # reset pulse remainder
        else:
            controlPeriod = fswRate

        for thrIdx in range(numThrusters):
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

    np.testing.assert_allclose(moduleOutput, trueVector, atol=1e-12, err_msg="OnTimeRequest does not match")


# This statement below ensures that the unitTestScript can be run as a stand-along python script
if __name__ == "__main__":
    test_thrFiringRemainderC(True, False, False)
