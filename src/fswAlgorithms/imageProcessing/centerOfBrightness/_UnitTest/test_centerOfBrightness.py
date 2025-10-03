
# ISC License
#
#  Copyright (c) 2025, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder
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

import cv2
import pytest, os, inspect, glob
import numpy as np

filename = inspect.getframeinfo(inspect.currentframe()).filename
path = os.path.dirname(os.path.abspath(filename))
bskName = 'Basilisk'
splitPath = path.split(bskName)

importErr = False
reasonErr = ""
try:
    from PIL import Image, ImageDraw
except ImportError:
    importErr = True
    reasonErr = "python Pillow package not installed---can't test CenterOfBrightness module"

from Basilisk.utilities import SimulationBaseClass
from Basilisk.utilities import macros
from Basilisk.architecture import messaging

try:
    from Basilisk.fswAlgorithms import centerOfBrightness
except ImportError:
    importErr = True
    reasonErr = "Center Of Brightness not built---check OpenCV option"


@pytest.mark.skipif(importErr, reason= reasonErr)
@pytest.mark.parametrize("image, blur,  save_test, valid_image, save_image, window_point_top_left, window_point_bottom_right",
                         [("full_circle.png", 1, False, False, False, [0, 0], [0, 0]),
                          ("full_circle.png", 1, False, True, False, [0, 0], [0, 0]),
                          ("test_circle.jpeg", 5, False, True, False, [0, 0], [0, 0]),
                          ("half_half.png", 1, True, True, False, [0, 0], [0, 0]),
                          ("half_half.png", 1, True, True, False, [50, 0], [275, 91])
                          ])

def test_module(show_plots, image, blur, save_test, valid_image, save_image, window_point_top_left, window_point_bottom_right):
    centerOfBrightnessTest(show_plots, image, blur, save_test, valid_image, save_image, window_point_top_left,
                           window_point_bottom_right)


def compute_window_center(window_point_top_left, window_point_bottom_right):
    center_x = int(window_point_top_left[0] + (window_point_bottom_right[0] - window_point_top_left[0])/2)
    center_y = int(window_point_top_left[1] + (window_point_bottom_right[1] - window_point_top_left[1])/2)

    return np.array([center_x, center_y])


def compute_window_size(window_point_top_left, window_point_bottom_right):
    width = int(window_point_bottom_right[0] - window_point_top_left[0])
    height = int(window_point_bottom_right[1] - window_point_top_left[1])

    return [width, height]


def centerOfBrightnessTest(show_plots, image, blur, save_test, valid_image, save_image, window_point_top_left,
                           window_point_bottom_right):
    imagePath = path + '/' + image
    imagePath_module = path + '/temp_' + image
    input_image = Image.open(imagePath)
    input_image.load()

    # setup simulation environment
    unit_test_sim = SimulationBaseClass.SimBaseClass()
    process_rate = macros.sec2nano(0.5)
    test_process = unit_test_sim.CreateNewProcess("unit_process")
    test_process.addTask(unit_test_sim.CreateNewTask("unit_task", process_rate))

#     # setup center of brightness module
    window_center = compute_window_center(window_point_top_left, window_point_bottom_right)
    window_width, window_height = compute_window_size(window_point_top_left, window_point_bottom_right)
    module_config = centerOfBrightness.CenterOfBrightness()
    module_config.modelTag = "centerOfBrightness"
    if window_center.all() != 0 and window_width != 0 and window_height != 0:
        module_config.setWindowCenter(window_center)
        module_config.setWindowSize(window_width, window_height)

    module_config.setRelativeBrightnessIncreaseThreshold(0.0)
    np.testing.assert_equal(module_config.getRelativeBrightnessIncreaseThreshold(), 0.0)
    module_config.setNumberOfPointsBrightnessAverage(3)
    np.testing.assert_equal(module_config.getNumberOfPointsBrightnessAverage(),3)
    module_config.setFileName(image_path_module)
    np.testing.assert_equal(module_config.getFileName(), image_path_module)
    module_config.setBlurSize(blur)
    np.testing.assert_equal(module_config.getBlurSize(), blur)
    module_config.setPixelThreshold(50)
    np.testing.assert_equal(module_config.getPixelThreshold(), 50)
    module_config.setSaveDir(path + '/result_save.png')
    np.testing.assert_equal(module_config.getSaveDir(), path + '/result_save.png')

    if save_test:
        module_config.setSaveImages(True)
        np.testing.assert_equal(module_config.getSaveImages(), True)

    unit_test_sim.AddModelToTask("unit_task", module_config)

    cob_ref = [input_image.width/2, input_image.height/2]
    if image == "half_half.png":
        # left half black, right half white, and a 1px wide grey stripe in the center with brightness 116/255
        white_width = 138
        grey_width = 1
        height = 183
        if np.array_equal(window_point_top_left, [50, 0]) and np.array_equal(window_point_bottom_right, [275, 91]):
            height = 91

        cob_ref = [(3/4 * 1 * white_width + 1/2 * 116/255 * grey_width)/(white_width + grey_width) * input_image.width,
                   int(height/2)*valid_image]
        pixelNum_ref = ((white_width+grey_width)*height)*valid_image

    inputMessageData = messaging.CameraImageMsgPayload()
    inputMessageData.timeTag = int(1E9)
    inputMessageData.cameraID = 1
    inputMessageData.valid = valid_image
    imgInMsg = messaging.CameraImageMsg().write(inputMessageData)
    module_config.imageInMsg.subscribeTo(imgInMsg)
    dataLog = module_config.opnavCOBOutMsg.recorder()
    unit_test_sim.AddModelToTask("unit_task", dataLog)

    unit_test_sim.InitializeSimulation()

    # run simulation for 5 time steps (excluding initial time step at 0 ns), scale brightness each time step
    # necessary to test rolling brightness average
    scaler = np.array([0.5, 0.6, 0.8, 0.3, 0.9])
    brightness_ref = np.zeros([len(scaler)])
    brightnessAverage_ref = np.zeros([len(scaler)])
    for i in range(0, len(scaler)):
        im = cv2.imread(image_path)
        th, im_th = cv2.threshold(im, int(scaler[i] * 255), 255, cv2.THRESH_TRUNC)
        cv2.imwrite(image_path_module, im_th)

        unit_test_sim.ConfigureStopTime(i * process_rate)
        unit_test_sim.ExecuteSimulation()

        # true rolling brightness average
        if image == "half_half.png":
            brightness_raw = int(scaler[i] * 255)*white_width*height + 116*grey_width*height
            brightness_ref[i] = brightness_raw / 255
            lower_idx = max(0, i-(int(module_config.getNumberOfPointsBrightnessAverage())-1))
            brightnessAverage_ref[i] = np.mean(brightness_ref[lower_idx:i+1])

    center = dataLog.centerOfBrightness[0, :]
    pixelNum = dataLog.pixelsFound[0]
    brightnessAverage = dataLog.rollingAverageBrightness

    output_image = Image.new("RGB", input_image.size)
    output_image.paste(input_image)
    draw_result = ImageDraw.Draw(output_image)

    if pixelNum > 0:
        data = [center[0], center[1], np.sqrt(pixelNum)/50]
        draw_result.ellipse((data[0] - data[2], data[1] - data[2], data[0] + data[2], data[1] + data[2]),
                            outline=(255, 0, 0, 0))
    if window_center.all() != 0 and window_width != 0 and window_height != 0:
        draw_result.rectangle((window_point_top_left[0], window_point_top_left[1], window_point_bottom_right[0],
                               window_point_bottom_right[1]), outline=(0, 255, 0, 0))

    input_image.close()

    # Remove saved images for the test of that functionality
    files = glob.glob(path + "/result_*")
    for f in files:
        os.remove(f)
    # Save output image with center of brightness
    if save_image:
        output_image.save("result_" + image)

    if show_plots:
        output_image.show()

    # Remove temporarily created images
    files = glob.glob(image_path_module)
    for f in files:
        os.remove(f)

    # make sure module output data is correct
    tolerance = 0.6  # just above half a pixel
    np.testing.assert_allclose(center,
                               cob_ref,
                               rtol=0,
                               atol=tolerance,
                               err_msg='Variable: rhat_COB_N',
                               verbose=True)

    if image == "half_half.png":
        np.testing.assert_allclose(pixelNum,
                                   pixelNum_ref,
                                   rtol=0,
                                   atol=tolerance,
                                   err_msg='Variable: pixelNum',
                                   verbose=True)

        np.testing.assert_allclose(brightnessAverage,
                                   brightnessAverage_ref,
                                   rtol=0.001,
                                   atol=0,
                                   err_msg='Variable: brightnessAverage',
                                   verbose=True)


if __name__ == "__main__":
    centerOfBrightnessTest(True, "half_half.png", 1, True, True, False, [50, 0], [275, 91])
