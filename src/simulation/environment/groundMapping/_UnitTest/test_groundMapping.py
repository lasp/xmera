# SPDX-License-Identifier: ISC
# Copyright (c) 2022, Autonomous Vehicle System Lab, University of Colorado at Boulder
# Copyright (c) 2025, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder
#

import math

import numpy as np
from xmera.architecture import messaging
from xmera.simulation import groundMapping
from xmera.utilities import SimulationBaseClass
from xmera.utilities import macros
from xmera.utilities import unitTestSupport


def test_groundMapping():
    r"""
    This test checks two points to determine if they are accessible for mapping or not. One point should be mapped,
    and one point should not be mapped.

    The inertial, planet-fixed planet-centered, and spacecraft body frames are all aligned.
    The spacecraft is in the -y direction of the inertial frame. The first point is along the line from the spacecraft
    to the origin. The second point is along the z-axis. The first point should be accessible because a.) the spacecraft
    is within the point's visibility cone and the point is within the spacecraft's visibility cone. The second point is
    not accessible because the spacecraft is not within the point's visibility cone and the point is not within the
    spacecraft's visibility cone.
    """
    [testResults, testMessage] = groundMappingTestFunction()
    assert testResults < 1, testMessage


def groundMappingTestFunction():
    """Test method"""
    testFailCount = 0
    testMessages = []
    unitTaskName = "unitTask"
    unitProcessName = "TestProcess"

    unitTestSim = SimulationBaseClass.SimBaseClass()
    testProcessRate = macros.sec2nano(0.5)
    testProc = unitTestSim.CreateNewProcess(unitProcessName)
    testProc.addTask(unitTestSim.CreateNewTask(unitTaskName, testProcessRate))

    # Configure blank module input messages
    planetInMsgData = messaging.SpicePlanetStateMsgPayload()
    planetInMsgData.J20002Pfix = [[1., 0., 0.], [0., 1., 0.], [0., 0., 1.]]
    planetInMsgData.PositionVector = [0., 0., 0.]
    planetInMsg = messaging.SpicePlanetStateMsg().write(planetInMsgData)

    scStateInMsgData = messaging.SCStatesMsgPayload()
    scStateInMsgData.r_BN_N = [0., -1., 0.]
    scStateInMsgData.sigma_BN = [0., 0., 0.]
    scStateInMsg = messaging.SCStatesMsg().write(scStateInMsgData)

    # Create the initial imaging target
    groundMap = groundMapping.GroundMapping()
    groundMap.modelTag = "groundMapping"
    groundMap.addPointToModel([0., -0.1, 0.])
    groundMap.addPointToModel([0., 0., math.tan(np.radians(22.5))+0.1])
    groundMap.minimumElevation = np.radians(45.)
    groundMap.maximumRange = 1e9
    groundMap.cameraPos_B = [0, 0, 0]
    groundMap.nHat_B = [0, 1, 0]
    groundMap.halfFieldOfView = np.radians(22.5)
    groundMap.scStateInMsg.subscribeTo(scStateInMsg)
    groundMap.planetInMsg.subscribeTo(planetInMsg)
    unitTestSim.AddModelToTask(unitTaskName, groundMap)

    # Setup the logging for the mapping locations
    mapLog = []
    for idx in range(0, 2):
        mapLog.append(groundMap.accessOutMsgs[idx].recorder())
        unitTestSim.AddModelToTask(unitTaskName, mapLog[idx])

    # subscribe input messages to module
    groundMap.planetInMsg.subscribeTo(planetInMsg)
    groundMap.scStateInMsg.subscribeTo(scStateInMsg)

    # setup output message recorder objects
    unitTestSim.InitializeSimulation()
    unitTestSim.ConfigureStopTime(macros.sec2nano(1.0))
    unitTestSim.ExecuteSimulation()

    # pull module data and make sure it is correct
    map_access = np.zeros(2, dtype=bool)
    for idx in range(0, 2):
        access = mapLog[idx].hasAccess
        if sum(access):
            map_access[idx] = 1

    # If the first target is not mapped, failure
    if not map_access[0]:
        testFailCount += 1

    # If the second target is mapped, failure
    if map_access[1]:
        testFailCount += 1

    if testFailCount == 0:
        print("PASSED: " + groundMap.modelTag)
    else:
        print(testMessages)

    return [testFailCount, "".join(testMessages)]


if __name__ == "__main__":
    test_groundMapping()
