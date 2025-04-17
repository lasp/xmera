
# ISC License
#
# Copyright (c) 2016, Autonomous Vehicle Systems Lab, University of Colorado at Boulder
#
# Permission to use, copy, modify, and/or distribute this software for any
# purpose with or without fee is hereby granted, provided that the above
# copyright notice and this permission notice appear in all copies.
#
# THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
# WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
# MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
# ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
# WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
# ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
# OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.


import numpy as np
import pytest
from Basilisk.architecture import messaging
from Basilisk.simulation import starTracker
from Basilisk.utilities import RigidBodyKinematics as rbk
from Basilisk.utilities import SimulationBaseClass
from Basilisk.utilities import macros
from Basilisk.utilities import unitTestSupport


def listStack(vec, simStopTime, unitProcRate):
    # returns a list duplicated the number of times needed to be consistent with module output
    return [vec] * int(simStopTime / (float(unitProcRate) / float(macros.sec2nano(1))))

def setRandomWalk(self, senNoiseStd = 0.0, errorBounds = [[1e6],[1e6],[1e6]]):
    # sets the module random walk variables
    PMatrix = [[senNoiseStd, 0., 0.], [0., senNoiseStd, 0.], [0., 0., senNoiseStd]]
    self.PMatrix = PMatrix
    self.walkBounds = errorBounds

# uncomment this line is this test is to be skipped in the global unit test run, adjust message as needed
# @pytest.mark.skipif(conditionstring)
# uncomment this line if this test has an expected failure, adjust message as needed
@pytest.mark.parametrize("useFlag, testCase", [
    (False, 'basic'),
    (False, 'noise'),
    (False, 'walk bounds')
])


def test_unitSimStarTracker(show_plots, useFlag, testCase):
    [testResults, testMessage] = unitSimStarTracker(show_plots, useFlag, testCase)
    assert testResults < 1, testMessage


def unitSimStarTracker(show_plots, useFlag, testCase):
    testFail = False
    testFailCount = 0
    testMessages = []
    unitTaskName = "unitTask"
    unitProcName = "TestProcess"

    # Create a sim module as an empty container
    unitSim = SimulationBaseClass.SimBaseClass()

    unitProcRate = macros.sec2nano(0.1)
    unitProcRate_s = macros.NANO2SEC*unitProcRate
    unitProc = unitSim.CreateNewProcess(unitProcName)
    unitProc.addTask(unitSim.CreateNewTask(unitTaskName, unitProcRate))

    # Configure the starTracker module
    StarTracker = starTracker.StarTracker()
    StarTracker.modelTag = "StarTracker"
    setRandomWalk(StarTracker)
    unitSim.AddModelToTask(unitTaskName, StarTracker)

    # Configure starTracker SCState input message
    OutputStateData = messaging.SCStatesMsgPayload()
    OutputStateData.r_BN_N = [0, 0, 0]
    OutputStateData.v_BN_N = [0, 0, 0]
    OutputStateData.sigma_BN = [0, 0, 0]
    OutputStateData.omega_BN_B = [0, 0, 0]
    OutputStateData.TotalAccumDVBdy = [0, 0, 0]
    OutputStateData.MRPSwitchCount = 0

    trueVector = dict()

    # This test verifies basic input and output
    if testCase == 'basic':
        simStopTime = 0.5
        sigma = np.array([-0.390614710591786, -0.503642740963740, 0.462959869561285])
        OutputStateData.sigma_BN = sigma
        trueVector['qInrtl2Case'] = listStack(rbk.MRP2EP(sigma),simStopTime,unitProcRate)
        trueVector['timeTag'] = np.arange(0, 0 + simStopTime*1E9, unitProcRate_s*1E9)

    elif testCase == 'noise':
        simStopTime = 1000.
        noiseStd = 0.1
        stdCorrectionFactor = 1.5  # This needs to be used because of the Gauss Markov module. need to fix the GM module
        setRandomWalk(StarTracker, noiseStd*stdCorrectionFactor, [[1.0e-13], [1.0e-13], [1.0e-13]])
        sigma = np.array([0, 0, 0])
        OutputStateData.sigma_BN = sigma
        trueVector['qInrtl2Case'] = [noiseStd] * 3
        trueVector['timeTag'] = np.arange(0, 0 + simStopTime*1E9, unitProcRate_s*1E9)

    # This test checks the walk bounds of random walk
    elif testCase == 'walk bounds':
        simStopTime = 1000.
        noiseStd = 0.01
        stdCorrectionFactor = 1.5  # This needs to be used because of the Gauss Markov module. need to fix the GM module
        walkBound = 0.1
        setRandomWalk(StarTracker, noiseStd*stdCorrectionFactor, [[walkBound], [walkBound], [walkBound]])
        sigma = np.array([0, 0, 0])
        OutputStateData.sigma_BN = sigma
        trueVector['qInrtl2Case'] = [walkBound + noiseStd*3] * 3
        trueVector['timeTag'] = np.arange(0, 0+simStopTime*1E9, unitProcRate_s*1E9)

    else:
        raise Exception('invalid test case')

    # Set up data logging
    dataLog = StarTracker.sensorOutMsg.recorder()
    unitSim.AddModelToTask(unitTaskName, dataLog)

    # Configure spacecraft state message
    scMsg = messaging.SCStatesMsg().write(OutputStateData)
    StarTracker.scStateInMsg.subscribeTo(scMsg)

    unitSim.InitializeSimulation()
    unitSim.ConfigureStopTime(macros.sec2nano(simStopTime))
    unitSim.ExecuteSimulation()

    # Extract logged data for test check
    moduleOutput = dataLog.qInrtl2Case

    # Convert quaternion output to prv
    moduleOutput2 = np.zeros([int(simStopTime/unitProcRate_s)+1, 3])
    for i in range(0, int(simStopTime/unitProcRate_s)+1):
        moduleOutput2[i] = rbk.EP2PRV(moduleOutput[i])

    if not 'accuracy' in vars():
        accuracy = 1e-6

    if testCase == 'noise':
        for i in range(0, 3):
            if np.abs(np.mean(moduleOutput2[:, i])) > 0.01 \
                            or np.abs(np.std(moduleOutput2[:, i]) - trueVector['qInrtl2Case'][i]) > 0.01:
                testFail = True
                break

    elif testCase == 'walk bounds':
        for i in range(0, 3):
            print(np.max(np.abs(np.asarray(moduleOutput2[i]))))
            if np.max(np.abs(np.asarray(moduleOutput2[i]))) > trueVector['qInrtl2Case'][i]:
                testFail = True
                break

    else:
        for i in range(0,len(trueVector['qInrtl2Case'])):
            if not unitTestSupport.isArrayEqual(moduleOutput[i], trueVector['qInrtl2Case'][i], 3, accuracy):
                testFail = True
                break

    if testFail:
        testFailCount += 1
        testMessages.append("FAILED: " + StarTracker.modelTag + " Module failed unit test")

    np.set_printoptions(precision=16)

    if testFailCount == 0:
        print("PASSED ")
    else:
        print(testMessages)

    return [testFailCount, ''.join(testMessages)]


if __name__ == "__main__":
    test_unitSimStarTracker(
        False,  # show_plots
        False,  # useFlag
        'walk bounds'  # testCase
    )
