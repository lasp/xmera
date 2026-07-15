# SPDX-License-Identifier: ISC
# Copyright (c) 2015, Autonomous Vehicle System Lab, University of Colorado at Boulder
# Copyright (c) 2024, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder
#

import inspect
import os

import numpy as np
import pytest

filename = inspect.getframeinfo(inspect.currentframe()).filename
path = os.path.dirname(os.path.abspath(filename))
bskName = 'xmera'
splitPath = path.split(bskName)

# Import all of the modules that we are going to be called in this simulation
importErr = False
reasonErr = ""
try:
    from PIL import Image, ImageDraw
except ImportError:
    importErr = True
    reasonErr = "python Pillow package not installed---can't test Cameras module"

# Import all of the modules that we are going to be called in this simulation
from xmera.utilities import SimulationBaseClass
from xmera.utilities import macros
from xmera.architecture import messaging
try:
    from xmera.simulation import camera
except ImportError:
    importErr = True
    reasonErr += "\nCamera not built---check OpenCV option"


# Uncomment this line is this test is to be skipped in the global unit test run, adjust message as needed.
# @pytest.mark.skipif(conditionstring)
# Uncomment this line if this test has an expected failure, adjust message as needed.
# @pytest.mark.xfail(conditionstring)
# Provide a unique test method name, starting with 'test_'.

@pytest.mark.skipif(importErr, reason=reasonErr)
@pytest.mark.parametrize("image, gauss, darkCurrent, saltPepper, cosmic, blurSize", [
    ("mars.jpg", 0, 0, 0, 0, 0),
    ("mars.jpg", 2, 2, 2, 1, 3)
])
def test_camera(show_plots, image, gauss, darkCurrent, saltPepper, cosmic, blurSize):
    """
    **Validation Test Description**

    This module tests the proper functioning of the camera module. This is done by first ensuring that the reading
    and writing of the camera parameters are properly executed. The test then corrupts a test image accordingly.

    **Description of Variables Being Tested**

    The camera parameters tested are the camera position MRP and the isOn value for the camera. These ensure that
    the position is properly written and read. The image is also corrupted with the parameterized test information.
    This is directly tested by differencing the initial and processed image to see a change.
    and also ensures that the variables are properly read and that all the openCV functions
    are executing properly.

    - ``camera_MRP``
    - ``isON``
    - ``imageNorm Values``

    The comparative value for the test on the image is 1E-2 which depends on the corruptions but is allowed to me small
    as the relative difference of the images is taken (whereas pixel values can get large).

    The two parameterized test are set with and without corruptions.

    **General Documentation Comments**

    The script could benefit from more profound image processing testing. Currently the bulk of the image processing
    is only tested by the result image.
    """
    if importErr:
        print(reasonErr)
        exit()

    # Truth values from python
    imagePath = path + '/' + image
    input_image = Image.open(imagePath)
    input_image.load()
    #################################################
    corrupted = (gauss > 0) or (darkCurrent > 0) or (saltPepper > 0) or (cosmic > 0) or (blurSize > 0)

    unitTaskName = "unitTask"  # arbitrary name (don't change)
    unitProcessName = "TestProcess"  # arbitrary name (don't change)

    # Create a sim module as an empty container
    unitTestSim = SimulationBaseClass.SimBaseClass()

    bitmapArray = []

    # # Create test thread
    testProcessRate = macros.sec2nano(0.5)  # update process rate update time
    testProc = unitTestSim.CreateNewProcess(unitProcessName)
    testProc.addTask(unitTestSim.CreateNewTask(unitTaskName, testProcessRate))

    # Construct algorithm and associated C++ container
    module = camera.Camera()
    module.modelTag = "cameras"

    # Add test module to runtime call list
    unitTestSim.AddModelToTask(unitTaskName, module)
    module.filename = imagePath
    module.saveImages = True
    # make each image saved have a unique name for this test case
    module.saveDir = '/'.join(imagePath.split('/')[:-1]) + '/' + str(gauss) + str(darkCurrent) \
                           + str(saltPepper) + str(cosmic) + str(blurSize)

    # Create input message and size it because the regular creator of that message
    # is not part of the test.
    inputMessageData = messaging.CameraImageMsgPayload()
    inputMessageData.timeTag = int(1E9)
    inputMessageData.cameraID = 1
    inCamMsg = messaging.CameraImageMsg().write(inputMessageData)
    module.imageInMsg.subscribeTo(inCamMsg)

    module.setCameraOn()
    module.setBodyToCameraMrp([0, 0, 1])
    module.setCameraBodyFramePosition([1, 1, 1])
    module.setParentName("name")
    module.setCameraId(1)
    module.setResolution([2,2])
    module.setFieldOfView([2.2, 2.2])
    module.setImageCadence(1)
    module.setFocalLength(2.1)
    module.setGaussianPointSpreadFunction(3)
    module.setReadNoise(2.3)
    module.setShotNoise(True)
    module.setDarkCurrent(darkCurrent)
    module.setSystemGain(3.3)
    module.setExposureTime(1.1)
    module.setGammaCorrection(1.1)
    module.setApertureRadius(0.1)
    module.setSensorWidth(0.5)
    module.setSensorHeight(0.6)
    module.setFullWellCapacity(1000.5)
    module.setIntegrationWeightFactor(1.1)
    module.setRedQuantumEfficiency([0.8, 0.7, 0.6])
    module.setGreenQuantumEfficiency([0.5, 0.6, 0.7])
    module.setBlueQuantumEfficiency([0.6, 0.7, 0.6])
    module.setHorizontalVignetting([0.1, 0.2, 0.3, 0.4])
    module.setVerticalVignetting([0.5, 0.6, 0.7, 0.8])
    module.setDistortion([0.2, 0.4, 0.6, 0.8])
    module.setTransmission(0.9)
    module.setImageFormat("png")
    module.setBitDepth(8)
    module.setDarkCurrentPattern(42)
    module.setDarkCurrentStdDeviation(0.02)
    module.setIsGrayscale(True)
    module.setPixelDefectPattern(7)
    module.setStuckPixelRate(0.01)
    module.setDeadPixelRate(0.03)

    # Noise parameters
    module.gaussian = gauss
    module.darkCurrentIntensity = darkCurrent
    module.saltPepper = saltPepper
    module.cosmicRays = cosmic
    module.blurParam = blurSize

    # Setup logging on the test module output message so that we get all the writes to it
    dataLog = module.cameraConfigOutMsg.recorder()
    unitTestSim.AddModelToTask(unitTaskName, dataLog)
    # For the CameraModelMsg
    dataLogCameraModel = module.cameraModelOutMsg.recorder()
    unitTestSim.AddModelToTask(unitTaskName, dataLogCameraModel)

    # Need to call the self-init and cross-init methods
    unitTestSim.InitializeSimulation()

    # Set the simulation time.
    # NOTE: the total simulation time may be longer than this value. The
    # simulation is stopped at the next logging event on or after the
    # simulation end time.
    unitTestSim.ConfigureStopTime(macros.sec2nano(0.5))  # seconds to stop simulation

    # Begin the simulation time run set above
    unitTestSim.ExecuteSimulation()

    # Truth values from python
    if corrupted:
        corruptedPath = module.saveDir + '0.000000.png'
    else:
        corruptedPath = module.saveDir + '0.500000.png'
    output_image = Image.open(corruptedPath)

    isOnValues = dataLog.isOn
    pos = dataLog.sigma_CB

    np.testing.assert_array_equal(dataLogCameraModel.cameraBodyFramePosition[-1, :],
                                  np.array(module.getCameraBodyFramePosition()).reshape(3),
                                  "Test failed camera position in the body frame")
    np.testing.assert_array_equal(dataLogCameraModel.bodyToCameraMrp[-1, :],
                                  np.array(module.getBodyToCameraMrp()).reshape(3),
                                  "Test failed MRP defining the orientation of the camera frame relative to the body frame")
    np.testing.assert_array_equal(dataLogCameraModel.resolution[-1, :], np.array(module.getResolution()).reshape(2),
                                  "Test failed camera resolution")
    np.testing.assert_array_equal(dataLogCameraModel.fieldOfView[-1, :], np.array(module.getFieldOfView()).reshape(2),
                                  "Test failed camera field of view")
    np.testing.assert_equal(dataLogCameraModel.cameraId, module.getCameraId(), "Test failed camera ID")
    np.testing.assert_equal(dataLogCameraModel.isOn, module.isCameraOn(), "Test failed isOn")
    np.testing.assert_equal(dataLogCameraModel.parentName, module.getParentName(),
                            "Test failed name of the parent body to which the camera should be attached")
    np.testing.assert_equal(dataLogCameraModel.renderRate, module.getImageCadence(),
                            "Test failed frame time interval at which to capture images")
    np.testing.assert_equal(dataLogCameraModel.focalLength, module.getFocalLength(),
                            "Test failed camera focal length")
    np.testing.assert_equal(dataLogCameraModel.gaussianPointSpreadFunction, module.getGaussianPointSpreadFunction(),
                            "Test failed size of square Gaussian kernel to model point spread function")
    np.testing.assert_equal(dataLogCameraModel.readNoise, module.getReadNoise(),
                            "Test failed read noise standard deviation")
    np.testing.assert_equal(dataLogCameraModel.shotNoise, module.getShotNoise(),
                            "Test failed shot noise")
    np.testing.assert_equal(dataLogCameraModel.darkCurrent, module.getDarkCurrent(),
                            "Test failed dark current")
    np.testing.assert_equal(dataLogCameraModel.systemGain, module.getSystemGain(),
                            "System failed mapping from current to pixel intensity")
    np.testing.assert_equal(dataLogCameraModel.exposureTime, module.getExposureTime(),
                            "System failed exposure time")
    np.testing.assert_equal(dataLogCameraModel.gammaCorrection, module.getGammaCorrection(),
                            "System failed gamma correction")
    np.testing.assert_equal(dataLogCameraModel.apertureRadius, module.getApertureRadius(),
                            "Test failed aperture radius")
    np.testing.assert_equal(dataLogCameraModel.sensorWidth, module.getSensorWidth(),
                            "Test failed sensor width")
    np.testing.assert_equal(dataLogCameraModel.sensorHeight, module.getSensorHeight(),
                            "Test failed sensor height")
    np.testing.assert_equal(dataLogCameraModel.fullWellCapacity, module.getFullWellCapacity(),
                            "Test failed full well capacity")
    np.testing.assert_equal(dataLogCameraModel.integrationWeightFactor, module.getIntegrationWeightFactor(),
                            "Test failed integration weight factor")
    np.testing.assert_array_equal(dataLogCameraModel.redQuantumEfficiency[-1, :],
                                  np.array(module.getRedQuantumEfficiency()).reshape(3),
                                  "Test failed red QE curve")
    np.testing.assert_array_equal(dataLogCameraModel.greenQuantumEfficiency[-1, :],
                                  np.array(module.getGreenQuantumEfficiency()).reshape(3),
                                  "Test failed green QE curve")
    np.testing.assert_array_equal(dataLogCameraModel.blueQuantumEfficiency[-1, :],
                                  np.array(module.getBlueQuantumEfficiency()).reshape(3),
                                  "Test failed blue QE curve")
    np.testing.assert_array_equal(np.array(dataLogCameraModel.horizontalVignetting[-1, :len(module.getHorizontalVignetting())]),
                                  np.array(module.getHorizontalVignetting()).reshape(-1),
                                  "Test failed horizontal Vignetting coefficients")
    np.testing.assert_array_equal(np.array(dataLogCameraModel.verticalVignetting[-1, :len(module.getVerticalVignetting())]),
                                  np.array(module.getVerticalVignetting()).reshape(-1),
                                  "Test failed vertical Vignetting coefficients")
    np.testing.assert_array_equal(np.array(dataLogCameraModel.distortion[-1, :len(module.getDistortion())]),
                                  np.array(module.getDistortion()).reshape(-1),
                                  "Test failed distortion coefficients")
    np.testing.assert_equal(dataLogCameraModel.transmission, module.getTransmission(),
                            "Test failed transmission rate of lens")
    np.testing.assert_equal(dataLogCameraModel.imageFormat, module.getImageFormat(),
                            "Test failed image format")
    np.testing.assert_equal(dataLogCameraModel.bitDepth, module.getBitDepth(),
                            "Test failed bit depth")
    np.testing.assert_equal(dataLogCameraModel.darkCurrentPattern, module.getDarkCurrentPattern(),
                            "Test failed dark current pattern seed")
    np.testing.assert_equal(dataLogCameraModel.darkCurrentStdDeviation, module.getDarkCurrentStdDeviation(),
                            "Test failed dark current standard deviation")
    np.testing.assert_equal(dataLogCameraModel.isGrayscale, module.getIsGrayscale(),
                            "Test failed grayscale flag")
    np.testing.assert_equal(dataLogCameraModel.pixelDefectPattern, module.getPixelDefectPattern(),
                            "Test failed pixel defect pattern seed")
    np.testing.assert_equal(dataLogCameraModel.stuckPixelRate, module.getStuckPixelRate(),
                            "Test failed stuck pixel rate")
    np.testing.assert_equal(dataLogCameraModel.deadPixelRate, module.getDeadPixelRate(),
                            "Test failed dead pixel rate")

    #  Error check for corruption
    err = np.linalg.norm(np.linalg.norm(input_image, axis=2) - np.linalg.norm(output_image, axis=2)) / np.linalg.norm(
        np.linalg.norm(input_image, axis=2))

    assert (err >= 1E-2) == corrupted, "Image error does not match corruption state for image: " + image

    #   print out success message if no error were found
    for i in range(3):
        assert np.abs(pos[-1, i] - module.cameraPos_B[i]) <= 1E-10, "Test failed position " + image

    assert np.abs(isOnValues[-1] - module.cameraIsOn) <= 1E-10, "Test failed isOn " + image

    # Clean up
    imagePath = path + '/' + image
    savedImage1 = '/'.join(imagePath.split('/')[:-1]) + '/' + str(gauss) + str(darkCurrent) \
                  + str(saltPepper) + str(cosmic) + str(blurSize) + '0.000000.png'
    savedImage2 = '/'.join(imagePath.split('/')[:-1]) + '/' + str(gauss) + str(darkCurrent) \
                  + str(saltPepper) + str(cosmic) + str(blurSize) + '0.500000.png'
    try:
        os.remove(savedImage1)
        os.remove(savedImage2)
    except FileNotFoundError:
        pass

#
# This statement below ensures that the unitTestScript can be run as a
# stand-along python script
#
if __name__ == "__main__":
    test_camera(False, "mars.jpg", 2, 0, 2, 1, 3)
