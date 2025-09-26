
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

def compute_brightness_and_pixel_refs(image, input_image,
                                      window_point_top_left, window_point_bottom_right,
                                      valid_image):

    cob_true = [input_image.width/2, input_image.height/2]
    pixel_num_true = None

    if image == "half_half.png":
        # left half black, right half white, and a 1px wide grey stripe in the center (116/255)
        white_width = 138
        grey_width = 1
        height = 183
        if np.array_equal(window_point_top_left, [50, 0]) and np.array_equal(window_point_bottom_right, [275, 91]):
            height = 91

        cob_true = [(3/4 * 1 * white_width + 1/2 * 116/255 * grey_width)/(white_width + grey_width) * input_image.width,
                   int(height/2)*valid_image]
        pixel_num_true = ((white_width+grey_width)*height)*valid_image

        return cob_true, pixel_num_true, white_width, grey_width, height

    return cob_true, pixel_num_true, None, None, None



def run_sequence(image, blur, save_test, valid_image,
                         window_point_top_left, window_point_bottom_right,
                         image_path, image_path_module):

    # setup simulation environment
    unit_test_sim = SimulationBaseClass.SimBaseClass()
    process_rate = macros.sec2nano(0.5)
    test_process = unit_test_sim.CreateNewProcess("unit_process")
    test_process.addTask(unit_test_sim.CreateNewTask("unit_task", process_rate))

#     # setup center of brightness module
    window_center = compute_window_center(window_point_top_left, window_point_bottom_right)
    window_width, window_height = compute_window_size(window_point_top_left, window_point_bottom_right)
    module_config = centerOfBrightness.CenterOfBrightness()
    module_config.modelTag = "centerOfBrightness_seq"
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

    input_message_data = messaging.CameraImageMsgPayload()
    input_message_data.timeTag = int(1E9)
    input_message_data.cameraID = 1
    input_message_data.valid = valid_image
    img_in_msg = messaging.CameraImageMsg().write(input_message_data)
    module_config.imageInMsg.subscribeTo(img_in_msg)

    data_log = module_config.opnavCOBOutMsg.recorder()
    unit_test_sim.AddModelToTask("unit_task", data_log)

    # run simulation for 5 time steps (excluding initial time step at 0 ns), scale brightness each time step
    # necessary to test rolling brightness average

    unit_test_sim.InitializeSimulation()
    scaler = np.array([0.5, 0.6, 0.8, 0.3, 0.9])
    brightness_ref = np.zeros([len(scaler)])
    brightnessAverage_ref = np.zeros([len(scaler)])


    white_width = grey_width = height = None
    if image == "half_half.png":
        white_width = 138
        grey_width = 1
        height = 183
        if np.array_equal(window_point_top_left, [50, 0]) and np.array_equal(window_point_bottom_right, [275, 91]):
            height = 91

    for i in range(0, len(scaler)):
        im = cv2.imread(image_path)
        th, im_th = cv2.threshold(im, int(scaler[i] * 255), 255, cv2.THRESH_TRUNC)
        cv2.imwrite(image_path_module, im_th)

        unit_test_sim.ConfigureStopTime(i * process_rate)
        unit_test_sim.ExecuteSimulation()

        if image == "half_half.png":
            brightness_raw = int(scaler[i] * 255)*white_width*height + 116*grey_width*height
            brightness_ref[i] = brightness_raw / 255
            lower_idx = max(0, i-(int(module_config.getNumberOfPointsBrightnessAverage())-1))
            brightnessAverage_ref[i] = np.mean(brightness_ref[lower_idx:i+1])

    cob = data_log.centerOfBrightness[0, :]
    pixelNum = data_log.pixelsFound[0]
    brightnessAverage = data_log.rollingAverageBrightness

    return cob, pixelNum, brightnessAverage, brightnessAverage_ref


def centerOfBrightnessTest(show_plots, image, blur, save_test, valid_image, save_image, window_point_top_left,
                           window_point_bottom_right):
    image_path = path + '/' + image
    image_path_module = path + '/temp_' + image
    input_image = Image.open(image_path)
    input_image.load()

    cob_true, pixel_num_true, _, _, _ = compute_brightness_and_pixel_refs(
        image, input_image, window_point_top_left, window_point_bottom_right, valid_image
    )

    cob, pixelNum, brightnessAverage, brightnessAverage_ref = run_sequence(
        image, blur, save_test, valid_image,
        window_point_top_left, window_point_bottom_right,
        image_path, image_path_module
    )

    window_center = compute_window_center(window_point_top_left, window_point_bottom_right)
    window_width, window_height = compute_window_size(window_point_top_left, window_point_bottom_right)

    output_image = Image.new("RGB", input_image.size)
    output_image.paste(input_image)
    draw_result = ImageDraw.Draw(output_image)

    if pixelNum > 0:
        data = [cob[0], cob[1], np.sqrt(pixelNum)/50]
        draw_result.ellipse((data[0] - data[2], data[1] - data[2], data[0] + data[2], data[1] + data[2]),
                            outline=(255, 0, 0, 0))
    if window_center.all() != 0 and window_width != 0 and window_height != 0:
        draw_result.rectangle((window_point_top_left[0], window_point_top_left[1], window_point_bottom_right[0],
                               window_point_bottom_right[1]), outline=(0, 255, 0, 0))

    input_image.close()

    files = glob.glob(path + "/result_*")
    for f in files:
        os.remove(f)
    if save_image:
        output_image.save("result_" + image)

    if show_plots:
        output_image.show()

    files = glob.glob(image_path_module)
    for f in files:
        os.remove(f)

    tolerance = 0.6  # just above half a pixel
    np.testing.assert_allclose(cob,
                               cob_true,
                               rtol=0,
                               atol=tolerance,
                               err_msg='Variable: rhat_COB_N',
                               verbose=True)

    if image == "half_half.png":
        np.testing.assert_allclose(pixelNum,
                                   pixel_num_true,
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

def test_windowing():

    image = "window_test.png"
    image_path = os.path.join(path, image)
    pixel_threshold = 50
    blur_size = 1
    atol_px = 0.6
    valid_image = True

    image_w = Image.open(image_path)
    w, h = image_w.size
    expected_full = np.array([w / 2, h / 2])

    image_gray = cv2.imread(image_path, cv2.IMREAD_GRAYSCALE)
    top_image = image_gray[0:h // 2, 0:w]
    top_bright_pixels = (top_image >= pixel_threshold).astype(np.float64)
    y, x = np.nonzero(top_bright_pixels)
    expected_top = np.array([x.mean(), y.mean()])
    
    # full image 
    unit_test_sim_full = SimulationBaseClass.SimBaseClass()
    process_rate = macros.sec2nano(0.5)
    test_process_full = unit_test_sim_full.CreateNewProcess("window_process_A")
    test_process_full.addTask(unit_test_sim_full.CreateNewTask("window_task_A", process_rate))

    module_config_full = centerOfBrightness.CenterOfBrightness()
    module_config_full.modelTag = "centerOfBrightness_window_full"
    module_config_full.setRelativeBrightnessIncreaseThreshold(0.0)
    module_config_full.setNumberOfPointsBrightnessAverage(1)
    module_config_full.setFileName(image_path)
    module_config_full.setBlurSize(blur_size)
    module_config_full.setPixelThreshold(pixel_threshold)
    module_config_full.setSaveDir(path + '/result_save.png')
    module_config_full.setSaveImages(False)

    input_message_data_full = messaging.CameraImageMsgPayload()
    input_message_data_full.timeTag = int(1E9); input_message_data_full.cameraID = 1;
    input_message_data_full.valid = valid_image
    imgIninput_message_data_full = messaging.CameraImageMsg().write(input_message_data_full)
    module_config_full.imageInMsg.subscribeTo(imgIninput_message_data_full)

    data_log_full = module_config_full.opnavCOBOutMsg.recorder()
    unit_test_sim_full.AddModelToTask("window_task_A", module_config_full)
    unit_test_sim_full.AddModelToTask("window_task_A", data_log_full)

    unit_test_sim_full.InitializeSimulation()
    unit_test_sim_full.ConfigureStopTime(process_rate)
    unit_test_sim_full.ExecuteSimulation()

    center_full = data_log_full.centerOfBrightness[0, :]

    # Top-half window
    unit_test_sim_window = SimulationBaseClass.SimBaseClass()
    test_process_window = unit_test_sim_window.CreateNewProcess("window_process_B")
    test_process_window.addTask(unit_test_sim_window.CreateNewTask("window_task_B", process_rate))

    module_config_window = centerOfBrightness.CenterOfBrightness()
    module_config_window.modelTag = "centerOfBrightness_window_test_top"
    module_config_window.setRelativeBrightnessIncreaseThreshold(0.0)
    module_config_window.setNumberOfPointsBrightnessAverage(1)
    module_config_window.setFileName(image_path)
    module_config_window.setBlurSize(blur_size)
    module_config_window.setPixelThreshold(pixel_threshold)
    module_config_window.setSaveDir(path + '/result_save.png')
    module_config_window.setSaveImages(False)

    # Define and set top-half
    x0 = 0
    y0 = 0
    x1 = w
    y1 = h // 2
    windowed_center = np.array([int((x0 + x1) / 2), int((y0 + y1) / 2)])
    windowed_size = (int(x1 - x0), int(y1 - y0))
    module_config_window.setWindowCenter(windowed_center)
    module_config_window.setWindowSize(windowed_size[0], windowed_size[1])

    input_message_data_window = messaging.CameraImageMsgPayload()
    input_message_data_window.timeTag = int(1E9); input_message_data_window.cameraID = 1;
    input_message_data_window.valid = valid_image
    imgIninput_message_data_window = messaging.CameraImageMsg().write(input_message_data_window)
    module_config_window.imageInMsg.subscribeTo(imgIninput_message_data_window)

    data_log_window = module_config_window.opnavCOBOutMsg.recorder()
    unit_test_sim_window.AddModelToTask("window_task_B", module_config_window)
    unit_test_sim_window.AddModelToTask("window_task_B", data_log_window)

    unit_test_sim_window.InitializeSimulation()
    unit_test_sim_window.ConfigureStopTime(process_rate)
    unit_test_sim_window.ExecuteSimulation()

    center_top = data_log_window.centerOfBrightness[0, :]

    np.testing.assert_allclose(
        center_full, expected_full, rtol=0.0, atol=atol_px,
        err_msg="COB mismatch for full-frame (no window)", verbose=True
    )
    np.testing.assert_allclose(
        center_top, expected_top, rtol=0.0, atol=atol_px,
        err_msg="COB mismatch for top-half window", verbose=True
    )

def test_threshold():
    image = "threshold_test.png"
    image_path = os.path.join(path, image)
    blur_size = 1
    atol_px = 0.6
    valid_image = True

    unit_test_sim_threshold = SimulationBaseClass.SimBaseClass()
    test_process_threshold = unit_test_sim_threshold.CreateNewProcess("threshold_process")
    process_rate = macros.sec2nano(0.5)
    test_process_threshold.addTask(unit_test_sim_threshold.CreateNewTask("threshold_task", process_rate))

    module_config_threshold = centerOfBrightness.CenterOfBrightness()
    module_config_threshold.modelTag = "centerOfBrightness_threshold_test"
    module_config_threshold.setRelativeBrightnessIncreaseThreshold(0.0)
    module_config_threshold.setNumberOfPointsBrightnessAverage(1)
    module_config_threshold.setFileName(image_path)
    module_config_threshold.setBlurSize(blur_size)
    module_config_threshold.setPixelThreshold(50)
    module_config_threshold.setSaveDir(path + '/result_save.png')
    module_config_threshold.setSaveImages(False)

    input_message_data_threshold = messaging.CameraImageMsgPayload()
    input_message_data_threshold.timeTag = int(1E9); input_message_data_threshold.cameraID = 1;
    input_message_data_threshold.valid = valid_image
    imgIninput_message_data_threshold = messaging.CameraImageMsg().write(input_message_data_threshold)
    module_config_threshold.imageInMsg.subscribeTo(imgIninput_message_data_threshold)

    data_log_threshold = module_config_threshold.opnavCOBOutMsg.recorder()
    unit_test_sim_threshold.AddModelToTask("threshold_task", module_config_threshold)
    unit_test_sim_threshold.AddModelToTask("threshold_task", data_log_threshold)

    unit_test_sim_threshold.InitializeSimulation()
    unit_test_sim_threshold.ConfigureStopTime(process_rate)
    unit_test_sim_threshold.ExecuteSimulation()

    cob_50 = data_log_threshold.centerOfBrightness[0, :]

# find cob for 100

    unit_test_sim_threshold_100 = SimulationBaseClass.SimBaseClass()
    test_process_threshold_100 = unit_test_sim_threshold_100.CreateNewProcess("threshold_100_process")
    process_rate = macros.sec2nano(0.5)
    test_process_threshold_100.addTask(unit_test_sim_threshold_100.CreateNewTask("threshold_100_task", process_rate))

    module_config_threshold_100 = centerOfBrightness.CenterOfBrightness()
    module_config_threshold_100.modelTag = "centerOfBrightness_threshold_100_test"
    module_config_threshold_100.setRelativeBrightnessIncreaseThreshold(0.0)
    module_config_threshold_100.setNumberOfPointsBrightnessAverage(1)
    module_config_threshold_100.setFileName(image_path)
    module_config_threshold_100.setBlurSize(blur_size)
    module_config_threshold_100.setPixelThreshold(100)
    module_config_threshold_100.setSaveDir(path + '/result_save.png')
    module_config_threshold_100.setSaveImages(False)

    input_message_data_threshold_100 = messaging.CameraImageMsgPayload()
    input_message_data_threshold_100.timeTag = int(1E9); input_message_data_threshold_100.cameraID = 1;
    input_message_data_threshold_100.valid = valid_image
    imgIninput_message_data_threshold_100 = messaging.CameraImageMsg().write(input_message_data_threshold_100)
    module_config_threshold_100.imageInMsg.subscribeTo(imgIninput_message_data_threshold_100)

    data_log_threshold_100 = module_config_threshold_100.opnavCOBOutMsg.recorder()
    unit_test_sim_threshold_100.AddModelToTask("threshold_100_task", module_config_threshold_100)
    unit_test_sim_threshold_100.AddModelToTask("threshold_100_task", data_log_threshold_100)

    unit_test_sim_threshold_100.InitializeSimulation()
    unit_test_sim_threshold_100.ConfigureStopTime(process_rate)
    unit_test_sim_threshold_100.ExecuteSimulation()

    cob_100 = data_log_threshold_100.centerOfBrightness[0, :]

    x_cob_50 = cob_50[0]
    x_cob_100 = cob_100[0]

    assert x_cob_100 > x_cob_50, "COB did not move right as threshold increased:"


if __name__ == "__main__":
    centerOfBrightnessTest(True, "half_half.png", 1, True, True, False, [50, 0], [275, 91])
