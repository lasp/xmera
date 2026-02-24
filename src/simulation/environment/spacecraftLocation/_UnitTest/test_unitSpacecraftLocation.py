# SPDX-License-Identifier: ISC
# Copyright (c) 2025, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder
#

import inspect
import os

import numpy as np
import pytest
from xmera.architecture import messaging
from xmera.simulation import spacecraftLocation
from xmera.utilities import RigidBodyKinematics as rbk
from xmera.utilities import SimulationBaseClass
from xmera.utilities import macros
from xmera.utilities import orbitalMotion
from xmera.utilities import unitTestSupport

filename = inspect.getframeinfo(inspect.currentframe()).filename
path = os.path.dirname(os.path.abspath(filename))
bskName = 'xmera'
splitPath = path.split(bskName)

@pytest.mark.parametrize("defaultPolarRadius", [False, True])
@pytest.mark.parametrize("defaultPlanet", [False, True])
@pytest.mark.parametrize("latitude", [55, 65, 115])
@pytest.mark.parametrize("maxRange", [-1, 3000.*1000])
@pytest.mark.parametrize("cone", [0, 1, -1])
def test_spacecraftLocation(show_plots, defaultPolarRadius, defaultPlanet, latitude, maxRange, cone):
    """
    Tests whether spacecraftLocation:

    1. defaults planet polar radius to equatorial radius if the polar radius is not set
    2. checks that the zero default planet states are used if the planet is not provided
    3. checks that the planet oblateness is accounted for
    4. checks if the optional sensor boresight axis is properly accounted for

    :return:
    """

    # each test method requires a single assert method to be called
    [testResults, testMessage] = run(show_plots, defaultPolarRadius, defaultPlanet, latitude, maxRange, cone)
    assert testResults < 1, testMessage


def run(showplots, defaultPolarRadius, defaultPlanet, latitude, maxRange, cone):

    testFailCount = 0  # zero unit test result counter
    testMessages = []  # create empty array to store test log messages

    simTaskName = "simTask"
    simProcessName = "simProcess"
    scSim = SimulationBaseClass.SimBaseClass()
    dynProcess = scSim.CreateNewProcess(simProcessName)
    simulationTimeStep = macros.sec2nano(1.)
    dynProcess.addTask(scSim.CreateNewTask(simTaskName, simulationTimeStep))

    #   Initialize new atmosphere and drag model, add them to task
    module = spacecraftLocation.SpacecraftLocation()
    module.modelTag = "scLocation"
    module.rEquator = orbitalMotion.REQ_EARTH * 1000.
    if not defaultPolarRadius:
        module.rPolar = orbitalMotion.REQ_EARTH * 1000. * 0.5
    if maxRange:
        module.maximumRange = maxRange
    if cone != 0:
        module.aHat_B = [0, 0, cone]
        module.theta = 80. * macros.D2R

    scSim.AddModelToTask(simTaskName, module)

    # create planet message
    planetPos = np.array([0.0, 0.0, 0.0])
    if not defaultPlanet:
        planet_message = messaging.SpicePlanetStateMsgPayload()
        planet_message.J20002Pfix = rbk.euler3(np.radians(-90.)).tolist()
        planetPos = np.array([orbitalMotion.AU * 1000, 0.0, 0.0])
        planet_message.PositionVector = planetPos
        planetMsg = messaging.SpicePlanetStateMsg().write(planet_message)
        module.planetInMsg.subscribeTo(planetMsg)

    # create primary spacecraft state message
    alt = 1000. * 1000      # meters
    scMsgData = messaging.SCStatesMsgPayload()
    r = orbitalMotion.REQ_EARTH * 1000. + alt
    scMsgData.r_BN_N = planetPos + np.array([r, 0.0, 0.0])
    scMsgData.sigma_BN = [1.0, 0.0, 0.0]
    scMsg = messaging.SCStatesMsg().write(scMsgData)
    module.primaryScStateInMsg.subscribeTo(scMsg)

    sc1MsgData = messaging.SCStatesMsgPayload()
    angle = np.radians(latitude)
    sc1MsgData.r_BN_N = planetPos + np.array([r * np.cos(angle), 0.0, r * np.sin(angle)])
    sc1Msg = messaging.SCStatesMsg().write(sc1MsgData)
    module.addSpacecraftToModel(sc1Msg)

    # Run the sim
    scSim.InitializeSimulation()

    # check polar planet radius default behavior
    if defaultPolarRadius and module.rPolar < 0:
        testFailCount += 1
        testMessages.append("FAILED: " + module.modelTag + " Module failed default polar radius check.")

    scSim.TotalSim.singleStepProcesses()

    accessMsg = module.accessOutMsgs[0].read()

    if latitude == 55 or (latitude == 65 and not defaultPolarRadius):
        trueAccess = 1
        if maxRange > 0:
            if accessMsg.slantRange > maxRange:
                trueAccess = 0
        if cone == 1:
            trueAccess = 0

        if accessMsg.hasAccess != trueAccess:
            testFailCount += 1
            testMessages.append("FAILED: " + module.modelTag + " Module failed access test.")

        if accessMsg.slantRange <= 1e-6:
            testFailCount += 1
            testMessages.append("FAILED: " + module.modelTag + " Module failed positive slant range test.")
    if (latitude == 65 and defaultPolarRadius) or latitude == 115:
        # should not have access
        if accessMsg.hasAccess != 0:
            testFailCount += 1
            testMessages.append("FAILED: " + module.modelTag + " Module failed negative have access test.")

        if np.abs(accessMsg.slantRange) > 1e-6:
            testFailCount += 1
            testMessages.append("FAILED: " + module.modelTag + " Module failed negative slant range test.")

    if testFailCount == 0:
        print("PASSED: " + module.modelTag)
    else:
        print(testMessages)

    return [testFailCount, ''.join(testMessages)]


if __name__ == '__main__':
    run(False
        , False      # defaultPolarRadius
        , True     # defaultPlanet
        , 55        # true latitude angle (deg)
        , -7000.*1000 # max range
        , 1            # cone case, 0-> no cone, 1 -> [0, 1, 0], -1 -> [0, -1, 0]
        )
