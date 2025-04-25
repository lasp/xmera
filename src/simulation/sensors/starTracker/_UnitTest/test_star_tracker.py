
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
def test_starTracker(show_plots, useFlag, testCase):
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
    scStatesMessageData = messaging.SCStatesMsgPayload()
    scStatesMessageData.r_BN_N = [0, 0, 0]
    scStatesMessageData.v_BN_N = [0, 0, 0]
    scStatesMessageData.sigma_BN = [0, 0, 0]
    scStatesMessageData.omega_BN_B = [0, 0, 0]
    scStatesMessageData.TotalAccumDVBdy = [0, 0, 0]
    scStatesMessageData.MRPSwitchCount = 0

    trueVector = dict()

    # This test verifies basic input and output
    if testCase == 'basic':
        simStopTime = 0.5
        sigma = np.array([-0.390614710591786, -0.503642740963740, 0.462959869561285])
        scStatesMessageData.sigma_BN = sigma
        trueVector['qInrtl2Case'] = listStack(rbk.MRP2EP(sigma),simStopTime,unitProcRate)
        trueVector['timeTag'] = np.arange(0, 0 + simStopTime*1E9, unitProcRate_s*1E9)

    elif testCase == 'noise':
        simStopTime = 1000.
        noiseStd = 0.1
        stdCorrectionFactor = 1.5  # This needs to be used because of the Gauss Markov module. need to fix the GM module
        setRandomWalk(StarTracker, noiseStd*stdCorrectionFactor, [[1.0e-13], [1.0e-13], [1.0e-13]])
        sigma = np.array([0, 0, 0])
        scStatesMessageData.sigma_BN = sigma
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
        scStatesMessageData.sigma_BN = sigma
        trueVector['qInrtl2Case'] = [walkBound + noiseStd*3] * 3
        trueVector['timeTag'] = np.arange(0, 0+simStopTime*1E9, unitProcRate_s*1E9)

    else:
        raise Exception('invalid test case')

    # Set up data logging
    starTrackerSensorMsgDataLog = StarTracker.sensorOutMsg.recorder()
    unitSim.AddModelToTask(unitTaskName, starTrackerSensorMsgDataLog)

    # Configure spacecraft state message
    scStatesMessage = messaging.SCStatesMsg().write(scStatesMessageData)
    StarTracker.scStateInMsg.subscribeTo(scStatesMessage)

    unitSim.InitializeSimulation()
    unitSim.ConfigureStopTime(macros.sec2nano(simStopTime))
    unitSim.ExecuteSimulation()

    # Extract logged data for test check
    beta_CN = starTrackerSensorMsgDataLog.qInrtl2Case

    # Convert quaternion output to prv
    prv_CN = np.zeros([int(simStopTime/unitProcRate_s)+1, 3])
    for i in range(0, int(simStopTime/unitProcRate_s)+1):
        prv_CN[i] = rbk.EP2PRV(beta_CN[i])

    if testCase == 'noise':
        boundArray = np.full((int(simStopTime/unitProcRate_s)+1), 0.01)
        for i in range(0, 3):
            np.testing.assert_array_less(np.abs(np.mean(prv_CN[:, i])),
                                         boundArray,
                                         verbose=True)

            np.testing.assert_array_less(np.abs(np.std(prv_CN[:, i]) - trueVector['qInrtl2Case'][i]),
                                         boundArray,
                                         verbose=True)
    elif testCase == 'walk bounds':
        for i in range(0, 3):
            np.testing.assert_array_less(np.max(np.abs(np.asarray(prv_CN[i]))),
                                         trueVector['qInrtl2Case'][i],
                                         verbose=True)
    else:
        accuracy = 1e-6
        for i in range(0, len(trueVector['qInrtl2Case'])):
            np.testing.assert_allclose(beta_CN[i],
                                       trueVector['qInrtl2Case'][i],
                                       atol=accuracy,
                                       verbose=True)

if __name__ == "__main__":
    test_starTracker(
        False,  # show_plots
        False,  # useFlag
        'walk bounds'  # testCase
    )
