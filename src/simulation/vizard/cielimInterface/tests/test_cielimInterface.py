# SPDX-License-Identifier: ISC
# Copyright (c) 2024, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder
#

import tempfile, sys
from xmera import __path__
bskPath = __path__[0]
sys.path.append(bskPath + "/../../build/simulation/vizard/cielimInterface/")
import cielimMessage_pb2
import delimited_protobuf
import numpy as np
import pytest
from xmera.architecture import messaging
from xmera.utilities import SimulationBaseClass

importErr = False
reasonErr = ""
try:
    from xmera.simulation import cielimInterface
except ImportError:
    importErr = True
    reasonErr = "\nCielim Interface not built ---check build option"

@pytest.mark.skipif(importErr, reason=reasonErr)
def test_interface():
    read_write_test()

def read_write_test():
    tmpdir = tempfile.TemporaryDirectory()
    unit_task_name = "unitTask"
    unit_process_name = "TestProcess"
    unit_test_sim = SimulationBaseClass.SimBaseClass()

    # Create test thread
    test_process_rate = int(1E9)  # update process rate update time
    test_process = unit_test_sim.CreateNewProcess(unit_process_name)
    test_process.addTask(unit_test_sim.CreateNewTask(unit_task_name, test_process_rate))

    module = cielimInterface.CielimInterface()
    module.modelTag = "cielim_interface"
    module.setOpNavMode(cielimInterface.ClosedLoopMode_OPEN_LOOP)
    module.setSaveFile(tmpdir.name + "/test_proto")

    unit_test_sim.AddModelToTask(unit_task_name, module)

    # Create epoch message
    epoch_payload = messaging.EpochMsgPayload()
    epoch_payload.year = 2050
    epoch_payload.month = 2
    epoch_payload.day = 15
    epoch_payload.hours = 22
    epoch_payload.minutes = 8
    epoch_payload.seconds = 58
    epoch_message = messaging.EpochMsg().write(epoch_payload)
    module.epochMessage.subscribeTo(epoch_message)

    # Create diagnostic message
    diagnostics_payload = messaging.ImageDiagnosticsPayload()
    diagnostics_payload.areaOfInterestCenter = [0,0]
    diagnostics_payload.areaOfInterestWidthHeight = [100,200]
    diagnostics_payload.threshold = 15/255
    diagnostic_message = messaging.ImageDiagnostics().write(diagnostics_payload)
    module.imageDiagnosticsMessage.subscribeTo(diagnostic_message)

    # Create camera rendering message
    rendering_payload = messaging.CameraRenderingMsgPayload()
    rendering_payload.cameraId = 1
    rendering_payload.cosmicRayStdDeviation = 10
    rendering_payload.strayLight = 1.0
    rendering_payload.starField = True
    rendering_payload.rendering = "Lumen"
    rendering_payload.smear = True
    rendering_payload.wavelengths = [450, 550, 650]
    rendering_message = messaging.CameraRenderingMsg().write(rendering_payload)
    module.cameraRenderingMessage.subscribeTo(rendering_message)

    # Create asteroid parameter message
    asteroid_parameter_payload = messaging.CelestialBodyParametersMsgPayload()
    asteroid_parameter_payload.bodyName = "bennu"
    asteroid_parameter_payload.shapeModel = "bennu_normalized"
    asteroid_parameter_payload.perlinNoiseOctaveCount = 3
    asteroid_parameter_payload.perlinNoiseBaseFrequency = 0.01
    asteroid_parameter_payload.perlinNoiseBaseAmplitude = 10.
    asteroid_parameter_payload.perlinNoisePersistence = 0.4
    asteroid_parameter_payload.proceduralRocks = 1.5
    asteroid_parameter_payload.brdf = "Lambertian"
    asteroid_parameter_payload.reflectanceParameters = [0.5]
    asteroid_parameter_payload.meanRadius = 10000
    asteroid_parameter_payload.geometricAlbedo = 0.1
    asteroid_parameter_payload.principalAxisDistortion = [1, 0.9, 1.1]
    asteroid_parameter_payload.sigma_BN = [0, 0, 0.5]

    # Create camera message
    camera_payload = messaging.CameraModelMsgPayload()
    camera_payload.timeTag = int(0.5*1E9)
    camera_payload.cameraId = 1
    camera_payload.parentName = "cielim_sat"
    camera_payload.fieldOfView = [20*np.pi/180, 15*np.pi/180]
    camera_payload.bodyToCameraMrp = [1/3,-1/3,-1/3]
    camera_payload.cameraBodyFramePosition = [1,1,1]
    camera_payload.resolution = [4000, 3000]
    camera_payload.renderRate = 10000
    camera_payload.focalLength = 0.1
    camera_payload.readNoise = 10
    camera_payload.shotNoise = True
    camera_payload.darkCurrent = 0.11
    camera_payload.systemGain = 1
    camera_payload.gaussianPointSpreadFunction = 5
    camera_payload.exposureTime = 0.1
    camera_payload.apertureRadius = 0.2
    camera_payload.sensorWidth = 0.5
    camera_payload.sensorHeight = 0.6
    camera_payload.fullWellCapacity = 1000.5
    camera_payload.integrationWeightFactor = 1.1
    camera_payload.gammaCorrection = 1.1
    camera_payload.redQuantumEfficiency = [0.8, 0.7, 0.6]
    camera_payload.greenQuantumEfficiency = [0.5, 0.6, 0.7]
    camera_payload.blueQuantumEfficiency = [0.6, 0.7, 0.6]
    camera_payload.horizontalVignetting = [0.1, 0.2, 0.3, 0.4]
    camera_payload.verticalVignetting = [0.5, 0.6, 0.7, 0.8]
    camera_payload.distortion = [0.2, 0.4, 0.6, 0.8]
    camera_payload.transmission = 0.9
    camera_message = messaging.CameraModelMsg().write(camera_payload)
    module.cameraModelMessage.subscribeTo(camera_message)

    # Create spacecraft message
    spacecraft_payload = messaging.SCStatesMsgPayload()
    spacecraft_payload.r_BN_N = [0,0,-5E5]
    spacecraft_payload.v_BN_N = [4, 5, 6]
    spacecraft_payload.sigma_BN = [np.sqrt(2)/2, 0, -np.sqrt(2)/2]
    spacecraft_message = messaging.SCStatesMsg().write(spacecraft_payload)
    module.spacecraftMessage.subscribeTo(spacecraft_message)

    # Create spice body messages using a similar structure to the factory (for ease of implementation)
    class GravBodies:
        planetName = ""
        isCentralBody = False
        planetBodyInMsg = messaging.SpicePlanetStateMsg()
        celestialParametersInMsg = messaging.CelestialBodyParametersMsg()

    grav_bodies = []
    bodies_message_list = []
    sun = GravBodies()
    sun.planetName = "sun"
    sun_data = messaging.SpicePlanetStateMsgPayload()
    sun_data.PositionVector = [10E10,0,0]
    sun_data.VelocityVector = [4, 5, 6]
    sun_data.J20002Pfix = np.eye(3)
    sun.planetBodyInMsg = messaging.SpicePlanetStateMsg().write(sun_data)
    grav_bodies.append(sun)
    bodies_message_list.append(sun_data)

    asteroid_bennu = GravBodies()
    asteroid_bennu.planetName = "bennu"
    asteroid_bennu.isCentralBody = True
    asteroid_data = messaging.SpicePlanetStateMsgPayload()
    asteroid_data.PositionVector = [0,0,0]
    asteroid_data.VelocityVector = [4, 5, 6]
    asteroid_data.J20002Pfix = -np.eye(3)
    asteroid_bennu.planetBodyInMsg = messaging.SpicePlanetStateMsg().write(asteroid_data)
    asteroid_bennu.celestialParametersInMsg = messaging.CelestialBodyParametersMsg().write(asteroid_parameter_payload)
    grav_bodies.append(asteroid_bennu)
    bodies_message_list.append(asteroid_data)

    for gravBody in grav_bodies:
        body = cielimInterface.SpiceBody()
        body.name = gravBody.planetName
        body.isCentralBody = gravBody.isCentralBody
        body.spiceStateMessage.subscribeTo(gravBody.planetBodyInMsg)
        if (gravBody.planetName == asteroid_parameter_payload.bodyName):
            body.celestialParametersMessage.subscribeTo(gravBody.celestialParametersInMsg)
        module.addCelestialBody(body)

    # Run block
    unit_test_sim.InitializeSimulation()
    unit_test_sim.ConfigureStopTime(int(1e9))
    unit_test_sim.ExecuteSimulation()
    module.closeProtobufFile()

    # Read the existing address book.
    file_handle = open(module.getSaveFilename(), "rb")
    cielim_message = delimited_protobuf.read(file_handle, cielimMessage_pb2.CielimMessage)

    np.testing.assert_equal(cielim_message.currentTime.frameNumber, 1)
    np.testing.assert_equal(cielim_message.currentTime.simTimeElapsed, test_process_rate)

    np.testing.assert_equal(cielim_message.epoch.year, epoch_payload.year)
    np.testing.assert_equal(cielim_message.epoch.month, epoch_payload.month)
    np.testing.assert_equal(cielim_message.epoch.day, epoch_payload.day)
    np.testing.assert_equal(cielim_message.epoch.hours, epoch_payload.hours)
    np.testing.assert_equal(cielim_message.epoch.minutes, epoch_payload.minutes)
    np.testing.assert_equal(cielim_message.epoch.seconds, epoch_payload.seconds)

    np.testing.assert_equal(cielim_message.spacecraft.spacecraftName, "cielim_sat")
    np.testing.assert_equal(cielim_message.spacecraft.position, spacecraft_payload.r_BN_N)
    np.testing.assert_equal(cielim_message.spacecraft.velocity, spacecraft_payload.v_BN_N)
    np.testing.assert_equal(cielim_message.spacecraft.attitude, spacecraft_payload.sigma_BN)

    np.testing.assert_equal(cielim_message.camera.cameraId, camera_payload.cameraId)
    np.testing.assert_equal(cielim_message.camera.parentName, "cielim_sat")
    np.testing.assert_equal(cielim_message.camera.cameraPositionInBody, camera_payload.cameraBodyFramePosition)
    np.testing.assert_equal(cielim_message.camera.bodyFrameToCameraMrp, camera_payload.bodyToCameraMrp)

    np.testing.assert_equal(cielim_message.camera.areaOfInterest.threshold, diagnostics_payload.threshold)
    np.testing.assert_equal(cielim_message.camera.areaOfInterest.centerX, diagnostics_payload.areaOfInterestCenter[0])
    np.testing.assert_equal(cielim_message.camera.areaOfInterest.centerY, diagnostics_payload.areaOfInterestCenter[1])
    np.testing.assert_equal(cielim_message.camera.areaOfInterest.width, diagnostics_payload.areaOfInterestWidthHeight[0])
    np.testing.assert_equal(cielim_message.camera.areaOfInterest.height, diagnostics_payload.areaOfInterestWidthHeight[1])

    np.testing.assert_equal(cielim_message.camera.lensModel.fieldOfView,camera_payload.fieldOfView)
    np.testing.assert_equal(cielim_message.camera.lensModel.focalLength, camera_payload.focalLength)
    np.testing.assert_equal(cielim_message.camera.lensModel.pointSpreadFunction, camera_payload.gaussianPointSpreadFunction)
    np.testing.assert_equal(cielim_message.camera.lensModel.apertureRadius, camera_payload.apertureRadius)
    np.testing.assert_equal(cielim_message.camera.lensModel.horizontalVignetting, camera_payload.horizontalVignetting)
    np.testing.assert_equal(cielim_message.camera.lensModel.verticalVignetting, camera_payload.verticalVignetting)
    np.testing.assert_equal(cielim_message.camera.lensModel.distortion, camera_payload.distortion)
    np.testing.assert_equal(cielim_message.camera.lensModel.transmission, camera_payload.transmission)

    np.testing.assert_equal(cielim_message.camera.sensorModel.resolution, camera_payload.resolution)
    np.testing.assert_equal(cielim_message.camera.sensorModel.renderRate, camera_payload.renderRate)
    np.testing.assert_equal(cielim_message.camera.sensorModel.exposureTime, camera_payload.exposureTime)
    np.testing.assert_equal(cielim_message.camera.sensorModel.readNoise, camera_payload.readNoise)
    np.testing.assert_equal(cielim_message.camera.sensorModel.shotNoise, camera_payload.shotNoise)
    np.testing.assert_equal(cielim_message.camera.sensorModel.darkCurrent, camera_payload.darkCurrent)
    np.testing.assert_equal(cielim_message.camera.sensorModel.systemGain, camera_payload.systemGain)
    np.testing.assert_equal(cielim_message.camera.sensorModel.sensorWidth, camera_payload.sensorWidth)
    np.testing.assert_equal(cielim_message.camera.sensorModel.sensorHeight, camera_payload.sensorHeight)
    np.testing.assert_equal(cielim_message.camera.sensorModel.fullWellCapacity, camera_payload.fullWellCapacity)
    np.testing.assert_equal(cielim_message.camera.sensorModel.gamma, camera_payload.gammaCorrection)

    np.testing.assert_equal(cielim_message.camera.sensorModel.qeCurve.integrationWeightFactor, camera_payload.integrationWeightFactor)
    np.testing.assert_equal(cielim_message.camera.sensorModel.qeCurve.redValue1, camera_payload.redQuantumEfficiency[0])
    np.testing.assert_equal(cielim_message.camera.sensorModel.qeCurve.redValue2, camera_payload.redQuantumEfficiency[1])
    np.testing.assert_equal(cielim_message.camera.sensorModel.qeCurve.redValue3, camera_payload.redQuantumEfficiency[2])
    np.testing.assert_equal(cielim_message.camera.sensorModel.qeCurve.greenValue1, camera_payload.greenQuantumEfficiency[0])
    np.testing.assert_equal(cielim_message.camera.sensorModel.qeCurve.greenValue2, camera_payload.greenQuantumEfficiency[1])
    np.testing.assert_equal(cielim_message.camera.sensorModel.qeCurve.greenValue3, camera_payload.greenQuantumEfficiency[2])
    np.testing.assert_equal(cielim_message.camera.sensorModel.qeCurve.blueValue1, camera_payload.blueQuantumEfficiency[0])
    np.testing.assert_equal(cielim_message.camera.sensorModel.qeCurve.blueValue2, camera_payload.blueQuantumEfficiency[1])
    np.testing.assert_equal(cielim_message.camera.sensorModel.qeCurve.blueValue3, camera_payload.blueQuantumEfficiency[2])

    np.testing.assert_equal(cielim_message.renderParameters.wavelength1, rendering_payload.wavelengths[0])
    np.testing.assert_equal(cielim_message.renderParameters.wavelength2, rendering_payload.wavelengths[1])
    np.testing.assert_equal(cielim_message.renderParameters.wavelength3, rendering_payload.wavelengths[2])
    np.testing.assert_equal(cielim_message.renderParameters.cosmicRayStdDeviation, rendering_payload.cosmicRayStdDeviation)
    np.testing.assert_equal(cielim_message.renderParameters.strayLight, rendering_payload.strayLight)
    np.testing.assert_equal(cielim_message.renderParameters.starField, rendering_payload.starField)
    np.testing.assert_equal(cielim_message.renderParameters.rendering, rendering_payload.rendering)
    np.testing.assert_equal(cielim_message.renderParameters.enableSmear, rendering_payload.smear)

    i = 0
    for message, body in zip(bodies_message_list, grav_bodies):
        name = body.planetName
        central = body.isCentralBody
        np.testing.assert_equal(cielim_message.celestialBodies[i].bodyName, name)
        np.testing.assert_equal(cielim_message.celestialBodies[i].position, message.PositionVector)
        np.testing.assert_equal(cielim_message.celestialBodies[i].velocity, message.VelocityVector)
        np.testing.assert_equal(cielim_message.celestialBodies[i].attitude, np.array(message.J20002Pfix).flatten())
        np.testing.assert_equal(cielim_message.celestialBodies[i].centralBody, central)
        if (name == asteroid_parameter_payload.bodyName):
            np.testing.assert_equal(cielim_message.celestialBodies[i].model.geometricAlbedo, asteroid_parameter_payload.geometricAlbedo)
            np.testing.assert_equal(cielim_message.celestialBodies[i].model.meanRadius, asteroid_parameter_payload.meanRadius)
            np.testing.assert_equal(cielim_message.celestialBodies[i].model.shapeModel, asteroid_parameter_payload.shapeModel)
            np.testing.assert_equal(cielim_message.celestialBodies[i].model.perlinNoise.octaveCount, asteroid_parameter_payload.perlinNoiseOctaveCount)
            np.testing.assert_equal(cielim_message.celestialBodies[i].model.perlinNoise.baseFrequency, asteroid_parameter_payload.perlinNoiseBaseFrequency)
            np.testing.assert_equal(cielim_message.celestialBodies[i].model.perlinNoise.baseAmplitude, asteroid_parameter_payload.perlinNoiseBaseAmplitude)
            np.testing.assert_equal(cielim_message.celestialBodies[i].model.perlinNoise.persistence, asteroid_parameter_payload.perlinNoisePersistence)
            np.testing.assert_equal(cielim_message.celestialBodies[i].model.refModel.brdfModel, asteroid_parameter_payload.brdf)
            np.testing.assert_equal(cielim_message.celestialBodies[i].model.refModel.reflectanceParameters, asteroid_parameter_payload.reflectanceParameters)
            np.testing.assert_equal(cielim_message.celestialBodies[i].model.refModel.isotropicScattering, asteroid_parameter_payload.isotropicScattering)
            np.testing.assert_equal(cielim_message.celestialBodies[i].model.proceduralRocks, asteroid_parameter_payload.proceduralRocks)
            np.testing.assert_equal(cielim_message.celestialBodies[i].model.principalAxisDistortion, asteroid_parameter_payload.principalAxisDistortion)
            np.testing.assert_equal(cielim_message.celestialBodies[i].model.inertialToBodyMrp, asteroid_parameter_payload.sigma_BN)
        i += 1

if __name__ == "__main__":
    test_interface()
