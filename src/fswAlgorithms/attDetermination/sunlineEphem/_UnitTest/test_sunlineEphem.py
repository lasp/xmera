#
#  ISC License
#
#  Copyright (c) 2016, Autonomous Vehicle Systems Lab, University of Colorado at Boulder
#
#  Permission to use, copy, modify, and/or distribute this software for any
#  purpose with or without fee is hereby granted, provided that the above
#  copyright notice and this permission notice appear in all copies.
#
#  THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
#  WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
#  MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
#  ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
#  WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
#  ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
#  OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
#

import numpy as np

from xmera.architecture import messaging
from xmera.fswAlgorithms import sunlineEphem
from xmera.utilities import SimulationBaseClass
from xmera.utilities import macros

def test_sunlineEphem():
    unitTaskName = "unitTask"               # arbitrary name (don't change)
    unitProcessName = "TestProcess"         # arbitrary name (don't change)

    # Create a sim module as an empty container
    unitTestSim = SimulationBaseClass.SimBaseClass()

    # Create test thread
    testProcessRate = macros.sec2nano(0.5)     # update process rate update time
    testProc = unitTestSim.CreateNewProcess(unitProcessName)
    testProc.addTask(unitTestSim.CreateNewTask(unitTaskName, testProcessRate))

    # Construct algorithm and associated C++ container
    sunlineEphemObj = sunlineEphem.SunlineEphem()
    sunlineEphemObj.modelTag = "sunlineEphem"           # update python name of test module

    # Add test module to runtime call list
    unitTestSim.AddModelToTask(unitTaskName, sunlineEphemObj)

    # Create input message and size it because the regular creator of that message
    # is not part of the test.

    vehAttData = messaging.NavAttMsgPayload()
    vehPosData = messaging.NavTransMsgPayload()
    sunData = messaging.EphemerisMsgPayload()


    # Artificially put sun at the origin.
    sunData.r_BdyZero_N = [0.0, 0.0, 0.0]
    vehAttInMsg = messaging.NavAttMsg().write(vehAttData)


    # Place spacecraft unit length away on each coordinate axis
    vehAttData.sigma_BN = [0.0, 0.0, 0.0]
    testVectors = [[-1.0, 0.0, 0.0],
                   [0.0, -1.0, 0.0],
                   [0.0, 0.0, -1.0],
                   [1.0, 0.0, 0.0],
                   [0.0, 1.0, 0.0],
                   [0.0, 0.0, 1.0],
                   [0.0, 0.0, 0.0]] # test if the space pos vector aligns with the sun pos vector

    estVector = np.zeros((len(testVectors), 3))

    vehPosInMsg = messaging.NavTransMsg()
    sunDataInMsg = messaging.EphemerisMsg().write(sunData)
    sunlineEphemObj.sunPositionInMsg.subscribeTo(sunDataInMsg)
    sunlineEphemObj.scPositionInMsg.subscribeTo(vehPosInMsg)
    sunlineEphemObj.scAttitudeInMsg.subscribeTo(vehAttInMsg)

    dataLog = sunlineEphemObj.navStateOutMsg.recorder()
    unitTestSim.AddModelToTask(unitTaskName, dataLog)

    for i in range(len(testVectors)):
        testVec = testVectors[i]
        vehPosData.r_BN_N = testVec
        vehPosInMsg.write(vehPosData)

        # Need to call the self-init and cross-init methods
        unitTestSim.InitializeSimulation()
        unitTestSim.ConfigureStopTime(macros.sec2nano(1.0))        # seconds to stop simulation
        unitTestSim.ExecuteSimulation()
        estVector[i] = dataLog.vehSunPntBdy[-1]

        # reset the module to test this functionality
        sunlineEphemObj.reset(1)


    # set the filtered output truth states
    trueVector = [
                [1.0, 0.0, 0.0],
                [0.0, 1.0, 0.0],
                [0.0, 0.0, 1.0],
                [-1.0, 0.0, 0.0],
                [0.0, -1.0, 0.0],
                [0.0, 0.0, -1.0],
                [0.0, 0.0, 0.0],
                ]

    # one assert; on failure numpy.testing will show exactly where the mismatch is
    for i in range(len(trueVector)):
        np.testing.assert_almost_equal(estVector[i], trueVector[i],
                                       decimal=12,
                                       verbose=False)


if __name__ == "__main__":
    test_sunlineEphem()
