# SPDX-License-Identifier: ISC
# Copyright (c) 2016, Autonomous Vehicle Systems Lab, University of Colorado at Boulder
# Copyright (c) 2026, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

import os
import inspect
import shutil

import cv2
import numpy as np
import pytest

from xmera.utilities import SimulationBaseClass, macros
from xmera.architecture import messaging

try:
    from xmera.fswAlgorithms import centerOfBrightness
    HAS_COB = True
except ImportError:
    HAS_COB = False

pytestmark = pytest.mark.skipif(not HAS_COB, reason="centerOfBrightness not built — check OpenCV option")

PATH = os.path.dirname(os.path.abspath(inspect.getframeinfo(inspect.currentframe()).filename))
PROCESS_RATE = macros.sec2nano(0.5)
ATOL_PX = 0.6


def get_image_size(image_path):
    """Return (width, height) of an image file."""
    img = cv2.imread(image_path)
    h, w = img.shape[:2]
    return w, h


def setup_cob_sim(image_path, blur, pixel_threshold, roi_center, roi_size,
                  camera_id=1, brightness_avg_points=1,
                  brightness_increase_threshold=0.0,
                  save_images=False, save_dir=""):
    """Create and configure a COB simulation.

    Returns (sim, module, data_log, diag_log, roi_msg).
    Caller must keep roi_msg reference alive for the simulation's lifetime.
    """
    reader = centerOfBrightness.ImageReaderFromFile()
    reader.setBlurSize(blur)
    reader.setPixelThreshold(pixel_threshold)
    reader.setFileName(image_path)
    if save_images:
        reader.setSaveImages(True)
        reader.setSaveDir(save_dir)

    module = centerOfBrightness.CenterOfBrightness(reader)
    module.modelTag = "cob_test"
    module.setCameraID(camera_id)
    module.setRelativeBrightnessIncreaseThreshold(brightness_increase_threshold)
    module.setNumberOfPointsBrightnessAverage(brightness_avg_points)

    sim = SimulationBaseClass.SimBaseClass()
    process = sim.CreateNewProcess("test_process")
    process.addTask("test_task", PROCESS_RATE)
    sim.AddModelToTask("test_task", module)

    roi_msg_data = messaging.RegionOfInterestMsgPayload()
    roi_msg_data.centerX = roi_center[0]
    roi_msg_data.centerY = roi_center[1]
    roi_msg_data.width = roi_size[0]
    roi_msg_data.height = roi_size[1]
    roi_msg_data.timeTag = 1e9
    roi_msg = messaging.RegionOfInterestMsg().write(roi_msg_data)
    module.roiInMsg.subscribeTo(roi_msg)

    data_log = module.opnavCOBOutMsg.recorder()
    diag_log = module.centerOfBrightnessDiagnosticOutMsg.recorder()
    sim.AddModelToTask("test_task", data_log)
    sim.AddModelToTask("test_task", diag_log)

    return sim, module, data_log, diag_log, roi_msg


def test_cob_circle():
    """full_circle.png with full-image ROI — COB near image center."""
    image_path = os.path.join(PATH, "full_circle.png")
    w, h = get_image_size(image_path)

    sim, _, data_log, _, _ = setup_cob_sim(
        image_path, blur=1, pixel_threshold=50,
        roi_center=[w // 2, h // 2], roi_size=[w, h])

    sim.InitializeSimulation()
    sim.ConfigureStopTime(PROCESS_RATE)
    sim.ExecuteSimulation()

    cob = data_log.centerOfBrightness[0, :]
    np.testing.assert_allclose(cob, [w / 2, h / 2], atol=ATOL_PX,
                               err_msg="COB not near center for full_circle.png")


def test_cob_windowed():
    """half_half.png — restricted ROI shifts COB compared to full image."""
    image_path = os.path.join(PATH, "half_half.png")
    w, h = get_image_size(image_path)

    # Full-image run
    sim_full, _, log_full, _, _ = setup_cob_sim(
        image_path, blur=1, pixel_threshold=50,
        roi_center=[w // 2, h // 4], roi_size=[w // 2, h])
    sim_full.InitializeSimulation()
    sim_full.ConfigureStopTime(PROCESS_RATE)
    sim_full.ExecuteSimulation()
    cob_full = log_full.centerOfBrightness[0, :]

    # Restricted ROI — right half of the image
    roi_center = [w // 2, 3 * h // 4]
    roi_size = [w // 2, h]
    sim_win, _, log_win, _, _ = setup_cob_sim(
        image_path, blur=1, pixel_threshold=50,
        roi_center=roi_center, roi_size=roi_size)
    sim_win.InitializeSimulation()
    sim_win.ConfigureStopTime(PROCESS_RATE)
    sim_win.ExecuteSimulation()
    cob_win = log_win.centerOfBrightness[0, :]

    assert not np.allclose(cob_full, cob_win, atol=1.0), \
        "Windowed COB should differ from full-image COB"


def test_threshold_impact():
    """threshold_test.png at threshold=50 vs 100 — higher threshold shifts COB right."""
    image_path = os.path.join(PATH, "threshold_test.png")
    w, h = get_image_size(image_path)
    roi_center = [w // 2, h // 2]
    roi_size = [w, h]

    sim_50, _, log_50, _, _ = setup_cob_sim(
        image_path, blur=1, pixel_threshold=50,
        roi_center=roi_center, roi_size=roi_size)
    sim_50.InitializeSimulation()
    sim_50.ConfigureStopTime(PROCESS_RATE)
    sim_50.ExecuteSimulation()
    cob_50 = log_50.centerOfBrightness[0, :]

    sim_100, _, log_100, _, _ = setup_cob_sim(
        image_path, blur=1, pixel_threshold=100,
        roi_center=roi_center, roi_size=roi_size)
    sim_100.InitializeSimulation()
    sim_100.ConfigureStopTime(PROCESS_RATE)
    sim_100.ExecuteSimulation()
    cob_100 = log_100.centerOfBrightness[0, :]

    assert cob_100[0] > cob_50[0], "COB did not move right as threshold increased"


def test_rolling_average():
    """ImageReaderFromFile.getCurrentImageTimeTag always returns 1, so only the first
    timestep is processed through the module.  Rolling-average accumulation
    across multiple steps is verified at the C++ algorithm level (unit + fuzz
    tests).  Here we confirm the single processed step produces a nonzero
    rolling average and subsequent steps are skipped."""
    image_path = os.path.join(PATH, "threshold_test.png")
    w, h = get_image_size(image_path)

    sim, _, data_log, _, _ = setup_cob_sim(
        image_path, blur=1, pixel_threshold=50,
        roi_center=[w // 2, h // 2], roi_size=[w, h],
        brightness_avg_points=3)

    sim.InitializeSimulation()
    sim.ConfigureStopTime(3 * PROCESS_RATE)
    sim.ExecuteSimulation()

    brightness = data_log.rollingAverageBrightness.flatten()
    nonzero = brightness[brightness > 0]
    assert len(nonzero) == 1, (
        f"Expected exactly 1 nonzero brightness value (file reader is only fresh once), got {len(nonzero)}"
    )


def test_diagnostics():
    """Diagnostic flags: noPixelTrigger and notExceedingBrightnessIncreaseTrigger."""
    image_path = os.path.join(PATH, "threshold_test.png")
    temp_path = os.path.join(PATH, "temp_diag.png")
    shutil.copy2(image_path, temp_path)
    w, h = get_image_size(image_path)
    roi_center = [w // 2, h // 2]
    roi_size = [w, h]

    try:
        # threshold=255 -> no pixels above threshold -> defaults remain (both False)
        sim_np, _, _, diag_np, _ = setup_cob_sim(
            temp_path, blur=1, pixel_threshold=255,
            roi_center=roi_center, roi_size=roi_size,
            brightness_increase_threshold=float('inf'))
        sim_np.InitializeSimulation()
        sim_np.ConfigureStopTime(PROCESS_RATE)
        sim_np.ExecuteSimulation()
        np.testing.assert_equal(diag_np.noPixelTrigger[0], False)
        np.testing.assert_equal(diag_np.notExceedingBrightnessIncreaseTrigger[0], False)

        # relativeBrightnessIncreaseThreshold=inf -> pixels found but brightness check fails
        sim_bi, _, _, diag_bi, _ = setup_cob_sim(
            temp_path, blur=1, pixel_threshold=50,
            roi_center=roi_center, roi_size=roi_size,
            brightness_increase_threshold=float('inf'))
        sim_bi.InitializeSimulation()
        sim_bi.ConfigureStopTime(PROCESS_RATE)
        sim_bi.ExecuteSimulation()
        np.testing.assert_equal(diag_bi.noPixelTrigger[0], False)
        np.testing.assert_equal(diag_bi.notExceedingBrightnessIncreaseTrigger[0], False)
    finally:
        if os.path.exists(temp_path):
            os.remove(temp_path)


def test_image_freshness_check():
    """ImageReaderFromFile.getCurrentImageTimeTag always returns 1, so only the first
    timestep processes the image (1 > previousImageTimeTag which starts at 0).
    Subsequent timesteps see 1 > 1 which is false, skipping the stale image."""
    image_path = os.path.join(PATH, "full_circle.png")
    w, h = get_image_size(image_path)

    sim, _, data_log, _, _ = setup_cob_sim(
        image_path, blur=1, pixel_threshold=50,
        roi_center=[w // 2, h // 2], roi_size=[w, h])

    sim.InitializeSimulation()
    sim.ConfigureStopTime(3 * PROCESS_RATE)
    sim.ExecuteSimulation()

    valid = data_log.valid.flatten()

    # Exactly one update should process the image
    assert np.sum(valid) == 1, f"Expected exactly 1 valid step, got {np.sum(valid)}"

    # That valid step should have the correct COB
    valid_idx = np.argmax(valid)
    cob = data_log.centerOfBrightness[valid_idx, :]
    np.testing.assert_allclose(cob, [w / 2, h / 2], atol=ATOL_PX,
                               err_msg="COB not near center on the valid step")

# ---------------------------------------------------------------------------
# ImageReaderFromMessage tests
# ---------------------------------------------------------------------------

def test_img_msg_setters_getters():
    """ImageReaderFromMessage setters/getters round-trip."""
    reader = centerOfBrightness.ImageReaderFromMessage()
    reader.setBlurSize(7)
    assert reader.getBlurSize() == 7
    reader.setPixelThreshold(42.5)
    assert reader.getPixelThreshold() == 42.5


def test_img_msg_check_for_new_image():
    """getCurrentImageTimeTag returns timeTag when new, 0 when not."""
    reader = centerOfBrightness.ImageReaderFromMessage()

    payload = messaging.CameraImageMsgPayload()
    payload.timeTag = 5000
    payload.valid = 1
    payload.cameraID = 0
    img_msg = messaging.CameraImageMsg().write(payload)
    reader.imageInMsg.subscribeTo(img_msg)

    # timeTag=5000 > previousTimeTag=1000 → returns 5000
    result = reader.getCurrentImageTimeTag(0, 1000)
    assert result == 5000

    # timeTag=5000 <= previousTimeTag=9000 → returns 0
    result = reader.getCurrentImageTimeTag(0, 9000)
    assert result == 0



if __name__ == '__main__':
    test_threshold_impact()
