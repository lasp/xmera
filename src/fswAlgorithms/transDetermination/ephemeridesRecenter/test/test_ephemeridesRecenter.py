# ISC License
#
# Copyright (c) 2025, Laboratory for Atmospheric Space Physics, University of Colorado at Boulder
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

import inspect
import os
import numpy as np

filename = inspect.getframeinfo(inspect.currentframe()).filename
path = os.path.dirname(os.path.abspath(filename))

from xmera.utilities import SimulationBaseClass
from xmera.fswAlgorithms import ephemeridesRecenter
from xmera.architecture import messaging

def test_body_recenter():
    """ Test ephemeridesRecenter. """
    mars_central_body()

def test_moon_recenter():
    """ Test ephemeridesRecenter. """
    moon_central_body()

def test_clear_recenter():
    """ Test ephemeridesRecenter. """
    clearing_values()

def mars_central_body():
    """ Test the ephemeridesRecenter module. Setup a simulation, """

    unitTaskName = "unitTask"
    unitProcessName = "TestProcess"

    unitTestSim = SimulationBaseClass.SimBaseClass()
    testProc = unitTestSim.CreateNewProcess(unitProcessName)
    testProc.addTask(unitTestSim.CreateNewTask(unitTaskName,  int(0.5e9)))

    ephemRecenter = ephemeridesRecenter.EphemeridesRecenter()
    ephemRecenter.modelTag = "ephemeridesRecenter"
    unitTestSim.AddModelToTask(unitTaskName, ephemRecenter)

    list_of_bodies = []
    # Create bodies to add to the module
    sunBody = ephemeridesRecenter.BodyEphemeris()
    sunInputPayload = messaging.EphemerisMsgPayload()
    position = [0,0,0]
    velocity = [0,0,0]
    sunInputPayload.r_BdyZero_N = position
    sunInputPayload.v_BdyZero_N = velocity
    sunInputPayload.timeTag = 1234.0
    sunInputMessage = messaging.EphemerisMsg().write(sunInputPayload)
    sunBody.inputEphemerisMsg.subscribeTo(sunInputMessage)
    sunBody.bodySpiceName = "sun"
    sunBody.originalCentralBodyName = "sun"
    list_of_bodies.append(sunBody)

    # Set this message
    earthBody = ephemeridesRecenter.BodyEphemeris()
    earthInputPayload = messaging.EphemerisMsgPayload()
    position = [1000, -200, 100]
    velocity = [10, 0, -8]
    earthInputPayload.r_BdyZero_N = position
    earthInputPayload.v_BdyZero_N = velocity
    earthInputPayload.timeTag = 1234.0
    earthInputMessage = messaging.EphemerisMsg().write(earthInputPayload)
    earthBody.inputEphemerisMsg.subscribeTo(earthInputMessage)
    earthBody.bodySpiceName = "earth"
    earthBody.originalCentralBodyName = "sun"
    list_of_bodies.append(earthBody)

    marsBody = ephemeridesRecenter.BodyEphemeris()
    marsInputPayload = messaging.EphemerisMsgPayload()
    position = [-4000, 3000, 10000]
    velocity = [-1, -2, 1]
    marsInputPayload.r_BdyZero_N = position
    marsInputPayload.v_BdyZero_N = velocity
    marsInputPayload.timeTag = 1234.0
    marsInputMessage = messaging.EphemerisMsg().write(marsInputPayload)
    marsBody.inputEphemerisMsg.subscribeTo(marsInputMessage)
    marsBody.bodySpiceName = "mars"
    marsBody.originalCentralBodyName = "sun"
    list_of_bodies.append(marsBody)

    moonBody = ephemeridesRecenter.BodyEphemeris()
    moonInputPayload = messaging.EphemerisMsgPayload()
    position = [-50, 30, 100]
    velocity = [-0.5, -0.2, 0.1]
    moonInputPayload.r_BdyZero_N = position
    moonInputPayload.v_BdyZero_N = velocity
    moonInputPayload.timeTag = 1234.0
    moonInputMessage = messaging.EphemerisMsg().write(moonInputPayload)
    moonBody.inputEphemerisMsg.subscribeTo(moonInputMessage)
    moonBody.bodySpiceName = "moon"
    moonBody.originalCentralBodyName = "earth"
    list_of_bodies.append(moonBody)

    dataLogList = list()
    names = []
    for body in list_of_bodies:
        ephemRecenter.addBodyEphemerisToRecenter(body)
        names.append(body.bodySpiceName)
    for body in list_of_bodies:
        index = ephemRecenter.getBodyIndexFromName(body.bodySpiceName)
        rec = ephemRecenter.recenteredEphemerisOutputMsgs[index].recorder()
        dataLogList.append(rec)
        unitTestSim.AddModelToTask(unitTaskName, rec)

    ephemRecenter.setPreviousCommonZeroBase("sun")
    ephemRecenter.setNewZeroBase("mars")
    unitTestSim.InitializeSimulation()
    unitTestSim.ConfigureStopTime(0)
    unitTestSim.ExecuteSimulation()

    mars_r_recentered = np.array(dataLogList[2].r_BdyZero_N).reshape([3,])
    mars_v_recentered = np.array(dataLogList[2].v_BdyZero_N).reshape([3,])
    mars_time_tag = dataLogList[2].timeTag

    np.testing.assert_array_almost_equal(mars_r_recentered, np.zeros(3), 15, "Mars position error")
    np.testing.assert_array_almost_equal(mars_v_recentered, np.zeros(3), 15, "Mars velocity error")
    np.testing.assert_equal(mars_time_tag, marsInputPayload.timeTag, "Mars time tag error")

    earth_r_recentered = np.array(dataLogList[1].r_BdyZero_N).reshape([3,])
    earth_v_recentered = np.array(dataLogList[1].v_BdyZero_N).reshape([3,])
    earth_time_tag = dataLogList[1].timeTag

    np.testing.assert_array_almost_equal(earth_r_recentered,
                                  np.array(earthInputPayload.r_BdyZero_N) - np.array(marsInputPayload.r_BdyZero_N),
                                         15,
                                  "Earth position error")
    np.testing.assert_array_almost_equal(earth_v_recentered,
                                  np.array(earthInputPayload.v_BdyZero_N) - np.array(marsInputPayload.v_BdyZero_N),
                                         15,
                                  "Earth velocity error")
    np.testing.assert_equal(earth_time_tag, earthInputPayload.timeTag, "Earth time tag error")

    sun_r_recentered = np.array(dataLogList[0].r_BdyZero_N).reshape([3,])
    sun_v_recentered = np.array(dataLogList[0].v_BdyZero_N).reshape([3,])
    sun_time_tag = dataLogList[0].timeTag

    np.testing.assert_array_almost_equal(sun_r_recentered,
                                  np.array(sunInputPayload.r_BdyZero_N) - np.array(marsInputPayload.r_BdyZero_N),
                                         15,
                                  "Sun position error")
    np.testing.assert_array_almost_equal(sun_v_recentered,
                                  np.array(sunInputPayload.v_BdyZero_N) - np.array(marsInputPayload.v_BdyZero_N),
                                         15,
                                  "Sun velocity error")
    np.testing.assert_equal(sun_time_tag, sunInputPayload.timeTag, "Sun time tag error")

    moon_r_recentered = np.array(dataLogList[3].r_BdyZero_N).reshape([3,])
    moon_v_recentered = np.array(dataLogList[3].v_BdyZero_N).reshape([3,])
    moon_time_tag = dataLogList[3].timeTag

    np.testing.assert_array_almost_equal(moon_r_recentered,
                                  np.array(earthInputPayload.r_BdyZero_N) - np.array(marsInputPayload.r_BdyZero_N)
                                  + np.array(moonInputPayload.r_BdyZero_N),
                                         15,
                                  "Moon position error")
    np.testing.assert_array_almost_equal(moon_v_recentered,
                                  np.array(earthInputPayload.v_BdyZero_N) - np.array(marsInputPayload.v_BdyZero_N)
                                  +np.array(moonInputPayload.v_BdyZero_N),
                                         15,
                                  "Moon velocity error")
    np.testing.assert_equal(moon_time_tag, moonInputPayload.timeTag, "Moon time tag error")

def moon_central_body():
    """ Test the ephemeridesRecenter module. Setup a simulation, """

    unitTaskName = "unitTask"
    unitProcessName = "TestProcess"

    unitTestSim = SimulationBaseClass.SimBaseClass()
    testProc = unitTestSim.CreateNewProcess(unitProcessName)
    testProc.addTask(unitTestSim.CreateNewTask(unitTaskName,  int(0.5e9)))

    ephemRecenter = ephemeridesRecenter.EphemeridesRecenter()
    ephemRecenter.modelTag = "ephemeridesRecenter"
    unitTestSim.AddModelToTask(unitTaskName, ephemRecenter)

    list_of_bodies = []
    # Create bodies to add to the module
    sunBody = ephemeridesRecenter.BodyEphemeris()
    sunInputPayload = messaging.EphemerisMsgPayload()
    position = [80000, 10000, -9000]
    velocity = [-5, 2, -0.9]
    sunInputPayload.r_BdyZero_N = position
    sunInputPayload.v_BdyZero_N = velocity
    sunInputPayload.timeTag = 1234.0
    sunInputMessage = messaging.EphemerisMsg().write(sunInputPayload)
    sunBody.inputEphemerisMsg.subscribeTo(sunInputMessage)
    sunBody.bodySpiceName = "sun"
    sunBody.originalCentralBodyName = "saturn"
    list_of_bodies.append(sunBody)

    # Set this message
    earthBody = ephemeridesRecenter.BodyEphemeris()
    earthInputPayload = messaging.EphemerisMsgPayload()
    position = [1000, -200, 100]
    velocity = [10, 0, -8]
    earthInputPayload.r_BdyZero_N = position
    earthInputPayload.v_BdyZero_N = velocity
    earthInputPayload.timeTag = 1234.0
    earthInputMessage = messaging.EphemerisMsg().write(earthInputPayload)
    earthBody.inputEphemerisMsg.subscribeTo(earthInputMessage)
    earthBody.bodySpiceName = "earth"
    earthBody.originalCentralBodyName = "saturn"
    list_of_bodies.append(earthBody)

    marsBody = ephemeridesRecenter.BodyEphemeris()
    marsInputPayload = messaging.EphemerisMsgPayload()
    position = [-4000, 3000, 10000]
    velocity = [-1, -2, 1]
    marsInputPayload.r_BdyZero_N = position
    marsInputPayload.v_BdyZero_N = velocity
    marsInputPayload.timeTag = 1234.0
    marsInputMessage = messaging.EphemerisMsg().write(marsInputPayload)
    marsBody.inputEphemerisMsg.subscribeTo(marsInputMessage)
    marsBody.bodySpiceName = "mars"
    marsBody.originalCentralBodyName = "saturn"
    list_of_bodies.append(marsBody)

    moonBody = ephemeridesRecenter.BodyEphemeris()
    moonInputPayload = messaging.EphemerisMsgPayload()
    position = [-50, 30, 100]
    velocity = [-0.5, -0.2, 0.1]
    moonInputPayload.r_BdyZero_N = position
    moonInputPayload.v_BdyZero_N = velocity
    moonInputPayload.timeTag = 1234.0
    moonInputMessage = messaging.EphemerisMsg().write(moonInputPayload)
    moonBody.inputEphemerisMsg.subscribeTo(moonInputMessage)
    moonBody.bodySpiceName = "moon"
    moonBody.originalCentralBodyName = "earth"
    list_of_bodies.append(moonBody)

    saturnBody = ephemeridesRecenter.BodyEphemeris()
    saturnInputPayload = messaging.EphemerisMsgPayload()
    position = [0,0,0]
    velocity = [0,0,0]
    saturnInputPayload.r_BdyZero_N = position
    saturnInputPayload.v_BdyZero_N = velocity
    saturnInputPayload.timeTag = 1234.0
    saturnInputMessage = messaging.EphemerisMsg().write(saturnInputPayload)
    saturnBody.inputEphemerisMsg.subscribeTo(saturnInputMessage)
    saturnBody.bodySpiceName = "saturn"
    saturnBody.originalCentralBodyName = "saturn"
    list_of_bodies.append(saturnBody)

    dataLogList = list()
    names = []
    for body in list_of_bodies:
        ephemRecenter.addBodyEphemerisToRecenter(body)
        names.append(body.bodySpiceName)
    for body in list_of_bodies:
        index = ephemRecenter.getBodyIndexFromName(body.bodySpiceName)
        rec = ephemRecenter.recenteredEphemerisOutputMsgs[index].recorder()
        dataLogList.append(rec)
        unitTestSim.AddModelToTask(unitTaskName, rec)

    ephemRecenter.setPreviousCommonZeroBase("saturn")
    ephemRecenter.setNewZeroBase("moon")
    unitTestSim.InitializeSimulation()
    unitTestSim.ConfigureStopTime(0)
    unitTestSim.ExecuteSimulation()

    moon_r_recentered = np.array(dataLogList[3].r_BdyZero_N).reshape([3,])
    moon_v_recentered = np.array(dataLogList[3].v_BdyZero_N).reshape([3,])
    moon_time_tag = dataLogList[3].timeTag

    np.testing.assert_array_almost_equal(moon_r_recentered, np.zeros(3), 15,"Moon position error")
    np.testing.assert_array_almost_equal(moon_v_recentered, np.zeros(3), 15, "Moon velocity error")
    np.testing.assert_equal(moon_time_tag, moonInputPayload.timeTag, "Moon time tag error")

    earth_r_recentered = np.array(dataLogList[1].r_BdyZero_N).reshape([3,])
    earth_v_recentered = np.array(dataLogList[1].v_BdyZero_N).reshape([3,])
    earth_time_tag = dataLogList[1].timeTag

    np.testing.assert_array_almost_equal(earth_r_recentered,
                                  - np.array(moonInputPayload.r_BdyZero_N),
                                         15,
                                  "Earth position error")
    np.testing.assert_array_almost_equal(earth_v_recentered,
                                  - np.array(moonInputPayload.v_BdyZero_N),
                                         15,
                                  "Earth velocity error")
    np.testing.assert_array_almost_equal(earth_time_tag, earthInputPayload.timeTag, 15, "Earth time tag error")

    sun_r_recentered = np.array(dataLogList[0].r_BdyZero_N).reshape([3,])
    sun_v_recentered = np.array(dataLogList[0].v_BdyZero_N).reshape([3,])
    sun_time_tag = dataLogList[0].timeTag

    np.testing.assert_array_almost_equal(sun_r_recentered,
                                  np.array(sunInputPayload.r_BdyZero_N) -
                                  (np.array(moonInputPayload.r_BdyZero_N) + np.array(earthInputPayload.r_BdyZero_N)),
                                         15,
                                  "Sun position error")
    np.testing.assert_array_almost_equal(sun_v_recentered,
                                  np.array(sunInputPayload.v_BdyZero_N) -
                                  (np.array(moonInputPayload.v_BdyZero_N) + np.array(earthInputPayload.v_BdyZero_N)),
                                         15,
                                  "Sun velocity error")
    np.testing.assert_equal(sun_time_tag, sunInputPayload.timeTag, "Sun time tag error")

    saturn_r_recentered = np.array(dataLogList[4].r_BdyZero_N).reshape([3,])
    saturn_v_recentered = np.array(dataLogList[4].v_BdyZero_N).reshape([3,])
    saturn_time_tag = dataLogList[3].timeTag

    np.testing.assert_array_almost_equal(saturn_r_recentered,
                                  np.array(saturnInputPayload.r_BdyZero_N) -
                                  (np.array(moonInputPayload.r_BdyZero_N) + np.array(earthInputPayload.r_BdyZero_N)),
                                         15,
                                  "Saturn position error")
    np.testing.assert_array_almost_equal(saturn_v_recentered,
                                  np.array(saturnInputPayload.v_BdyZero_N) -
                                  (np.array(moonInputPayload.v_BdyZero_N) + np.array(earthInputPayload.v_BdyZero_N)),
                                         15,
                                  "Saturn velocity error")
    np.testing.assert_equal(saturn_time_tag, saturnInputPayload.timeTag, "Saturn time tag error")

def clearing_values():
    unitTaskName = "unitTask"
    unitProcessName = "TestProcess"

    unitTestSim = SimulationBaseClass.SimBaseClass()
    testProc = unitTestSim.CreateNewProcess(unitProcessName)
    testProc.addTask(unitTestSim.CreateNewTask(unitTaskName,  int(0.5e9)))

    ephemRecenter = ephemeridesRecenter.EphemeridesRecenter()
    ephemRecenter.modelTag = "ephemeridesRecenter"
    unitTestSim.AddModelToTask(unitTaskName, ephemRecenter)

    list_of_bodies = []
    # Create bodies to add to the module
    sunBody = ephemeridesRecenter.BodyEphemeris()
    sunInputPayload = messaging.EphemerisMsgPayload()
    position = [0,0,0]
    velocity = [0,0,0]
    sunInputPayload.r_BdyZero_N = position
    sunInputPayload.v_BdyZero_N = velocity
    sunInputPayload.timeTag = 1234.0
    sunInputMessage = messaging.EphemerisMsg().write(sunInputPayload)
    sunBody.inputEphemerisMsg.subscribeTo(sunInputMessage)
    sunBody.bodySpiceName = "sun"
    sunBody.originalCentralBodyName = "sun"
    list_of_bodies.append(sunBody)

    # Set this message
    earthBody = ephemeridesRecenter.BodyEphemeris()
    earthInputPayload = messaging.EphemerisMsgPayload()
    position = [1000, -200, 100]
    velocity = [10, 0, -8]
    earthInputPayload.r_BdyZero_N = position
    earthInputPayload.v_BdyZero_N = velocity
    earthInputPayload.timeTag = 1234.0
    earthInputMessage = messaging.EphemerisMsg().write(earthInputPayload)
    earthBody.inputEphemerisMsg.subscribeTo(earthInputMessage)
    earthBody.bodySpiceName = "earth"
    earthBody.originalCentralBodyName = "sun"
    list_of_bodies.append(earthBody)

    marsBody = ephemeridesRecenter.BodyEphemeris()
    marsInputPayload = messaging.EphemerisMsgPayload()
    position = [-4000, 3000, 10000]
    velocity = [-1, -2, 1]
    marsInputPayload.r_BdyZero_N = position
    marsInputPayload.v_BdyZero_N = velocity
    marsInputPayload.timeTag = 1234.0
    marsInputMessage = messaging.EphemerisMsg().write(marsInputPayload)
    marsBody.inputEphemerisMsg.subscribeTo(marsInputMessage)
    marsBody.bodySpiceName = "mars"
    marsBody.originalCentralBodyName = "sun"
    list_of_bodies.append(marsBody)

    moonBody = ephemeridesRecenter.BodyEphemeris()
    moonInputPayload = messaging.EphemerisMsgPayload()
    position = [-50, 30, 100]
    velocity = [-0.5, -0.2, 0.1]
    moonInputPayload.r_BdyZero_N = position
    moonInputPayload.v_BdyZero_N = velocity
    moonInputPayload.timeTag = 1234.0
    moonInputMessage = messaging.EphemerisMsg().write(moonInputPayload)
    moonBody.inputEphemerisMsg.subscribeTo(moonInputMessage)
    moonBody.bodySpiceName = "moon"
    moonBody.originalCentralBodyName = "earth"
    list_of_bodies.append(moonBody)

    names = []
    for body in list_of_bodies:
        ephemRecenter.addBodyEphemerisToRecenter(body)
        names.append(body.bodySpiceName)

    ephemRecenter.setPreviousCommonZeroBase("sun")
    ephemRecenter.setNewZeroBase("mars")

    np.testing.assert_equal(ephemRecenter.getNumberOfBodies(), len(list_of_bodies), "Wrong number of bodies")
    np.testing.assert_equal(ephemRecenter.getAllNames()[:ephemRecenter.getNumberOfBodies()], names, "Wrong names for bodies")

    np.testing.assert_equal(ephemRecenter.getNewZeroBase(), "mars", "Wrong new zero base")
    np.testing.assert_equal(ephemRecenter.getPreviousCommonZeroBase(), "sun", "Wrong previous zero base")

    ephemRecenter.clearAllBodies()
    np.testing.assert_equal(ephemRecenter.getNumberOfBodies(), 0, "Clear bodies did no work")



if __name__ == '__main__':
    mars_central_body()
