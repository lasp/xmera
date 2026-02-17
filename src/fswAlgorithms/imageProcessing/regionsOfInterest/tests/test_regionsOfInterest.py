# SPDX-License-Identifier: ISC
# Copyright (c) 2026, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

import inspect
import os
import pytest
import numpy as np
import matplotlib.pyplot as plt

from xmera.architecture import messaging
from xmera.fswAlgorithms import regionsOfInterest
from xmera.utilities import SimulationBaseClass
from xmera.utilities import macros

filename = inspect.getframeinfo(inspect.currentframe()).filename
path = os.path.dirname(os.path.abspath(filename))

# Test tolerances
POSITION_TOLERANCE = 1  # pixel
PIXEL_COUNT_TOLERANCE = 5  # number of pixels

@pytest.mark.parametrize('num_regions, use_windowing', [
    (1, False),
    (2 , False),
    (3, False),
    (3, True),
])
def test_region_identification(show_plots, num_regions, use_windowing):
    """Test region identification with various configurations"""
    region_identification(show_plots, num_regions, use_windowing)


def test_single_region_basic(show_plots):
    """Test with a single bright region"""
    task_name = "unitTask"
    process_name = "TestProcess"

    # Create simulation
    sim = SimulationBaseClass.SimBaseClass()
    test_process = sim.CreateNewProcess(process_name)
    test_process.addTask(sim.CreateNewTask(task_name, macros.sec2nano(0.1)))

    # Create module
    roi_module = regionsOfInterest.RegionsOfInterest()
    roi_module.modelTag = "roi_module"
    sim.AddModelToTask(task_name, roi_module)

    # Configure module
    roi_module.setMaxRoiSeparation(100)

    # Create input message with single region
    regions_msg_payload = messaging.RegionsIdentifiedMsgPayload()
    region1 = messaging.RegionOfInterestMsgPayload()
    region1.numberOfPixels = 250
    region1.centerOfBrightnessX = 512
    region1.centerOfBrightnessY = 384
    region1.centerX = 512
    region1.centerY = 384
    region1.width = 20
    region1.height = 20
    region1.timeTag = 0.0

    region2 = messaging.RegionOfInterestMsgPayload()
    region3 = messaging.RegionOfInterestMsgPayload()

    # regions_msg_payload.regions[0].numberOfPixels = 250
    regions_msg_payload.regions = [region2, region1, region3]

    regions_input_msg = messaging.RegionsIdentifiedMsg().write(regions_msg_payload)
    roi_module.roisInMsg.subscribeTo(regions_input_msg)

    # Set up logging
    roi_log = roi_module.regionOutMsg.recorder()
    sim.AddModelToTask(task_name, roi_log)

    # Run simulation
    sim.InitializeSimulation()
    sim.ConfigureStopTime(macros.sec2nano(1.0))
    sim.ExecuteSimulation()

    # Verify output
    output_pixels = roi_log.numberOfPixels[-1]
    output_center_x = roi_log.centerX[-1]
    output_center_y = roi_log.centerY[-1]

    np.testing.assert_allclose(output_pixels, 250, rtol=0, atol=PIXEL_COUNT_TOLERANCE,
                               err_msg="Output pixel count incorrect")
    np.testing.assert_allclose(output_center_x, 512, rtol=0, atol=POSITION_TOLERANCE,
                               err_msg="Output center X incorrect")
    np.testing.assert_allclose(output_center_y, 384, rtol=0, atol=POSITION_TOLERANCE,
                               err_msg="Output center Y incorrect")


def test_zero_regions(show_plots):
    """Test with no detected regions"""
    task_name = "unitTask"
    process_name = "TestProcess"

    sim = SimulationBaseClass.SimBaseClass()
    test_process = sim.CreateNewProcess(process_name)
    test_process.addTask(sim.CreateNewTask(task_name, macros.sec2nano(0.1)))

    roi_module = regionsOfInterest.RegionsOfInterest()
    roi_module.modelTag = "roi_module"
    sim.AddModelToTask(task_name, roi_module)

    # Create empty input message
    regions_msg = messaging.RegionsIdentifiedMsgPayload()
    # All regions have zero pixels (default initialization)

    regions_input_msg = messaging.RegionsIdentifiedMsg().write(regions_msg)
    roi_module.roisInMsg.subscribeTo(regions_input_msg)

    roi_log = roi_module.regionOutMsg.recorder()
    sim.AddModelToTask(task_name, roi_log)

    sim.InitializeSimulation()
    sim.ConfigureStopTime(macros.sec2nano(1.0))
    sim.ExecuteSimulation()

    # Verify output is empty
    output_pixels = roi_log.numberOfPixels[-1]
    np.testing.assert_allclose(output_pixels, 0, rtol=0, atol=1,
                               err_msg="Output should be empty with no input regions")


def test_region_merging(show_plots):
    """Test merging of close regions"""
    task_name = "unitTask"
    process_name = "TestProcess"

    sim = SimulationBaseClass.SimBaseClass()
    test_process = sim.CreateNewProcess(process_name)
    test_process.addTask(sim.CreateNewTask(task_name, macros.sec2nano(0.1)))

    roi_module = regionsOfInterest.RegionsOfInterest()
    roi_module.modelTag = "roi_module"
    sim.AddModelToTask(task_name, roi_module)

    # Set small separation to force merging
    roi_module.setMaxRoiSeparation(50)

    # Create two close regions
    regions_msg_payload = messaging.RegionsIdentifiedMsgPayload()

    # Create input message with single region
    region1 = messaging.RegionOfInterestMsgPayload()
    region1.numberOfPixels = 100
    region1.centerOfBrightnessX = 500
    region1.centerOfBrightnessY = 400
    region1.centerX = 500
    region1.centerY = 400
    region2 = messaging.RegionOfInterestMsgPayload()
    region2.numberOfPixels = 80
    region2.centerOfBrightnessX = 510
    region2.centerOfBrightnessY = 405
    region2.centerX = 510
    region2.centerY = 405
    region3 = messaging.RegionOfInterestMsgPayload()

    regions_msg_payload.regions = [region1, region2, region3]

    regions_input_msg = messaging.RegionsIdentifiedMsg().write(regions_msg_payload)
    roi_module.roisInMsg.subscribeTo(regions_input_msg)

    roi_log = roi_module.regionOutMsg.recorder()
    sim.AddModelToTask(task_name, roi_log)

    sim.InitializeSimulation()
    sim.ConfigureStopTime(macros.sec2nano(1.0))
    sim.ExecuteSimulation()

    # Verify regions were merged (total pixel count = 180)
    output_pixels = roi_log.numberOfPixels[-1]
    np.testing.assert_allclose(output_pixels, 180, rtol=0, atol=PIXEL_COUNT_TOLERANCE,
                               err_msg="Regions should have been merged")

    # Verify center is weighted barycenter
    expected_center_x = (100 * 500 + 80 * 510) / 180
    expected_center_y = (100 * 400 + 80 * 405) / 180

    output_center_x = roi_log.centerOfBrightnessX[-1]
    output_center_y = roi_log.centerOfBrightnessY[-1]

    np.testing.assert_allclose(output_center_x, expected_center_x, rtol=0, atol=POSITION_TOLERANCE,
                               err_msg="Merged center X incorrect")
    np.testing.assert_allclose(output_center_y, expected_center_y, rtol=0, atol=POSITION_TOLERANCE,
                               err_msg="Merged center Y incorrect")


def test_windowing(show_plots):
    """Test windowing functionality"""
    task_name = "unitTask"
    process_name = "TestProcess"

    sim = SimulationBaseClass.SimBaseClass()
    test_process = sim.CreateNewProcess(process_name)
    test_process.addTask(sim.CreateNewTask(task_name, macros.sec2nano(0.1)))

    roi_module = regionsOfInterest.RegionsOfInterest()
    roi_module.modelTag = "roi_module"
    sim.AddModelToTask(task_name, roi_module)

    # Set up window centered at (512, 384) with size 400x300
    roi_module.setWindowCenter([512, 384])
    roi_module.setWindowSize(400, 300)

    # Create regions: one inside window, one outside
    regions_msg_payload = messaging.RegionsIdentifiedMsgPayload()

    # Create input message with single region
    region1 = messaging.RegionOfInterestMsgPayload()
    region1.numberOfPixels = 100
    region1.centerOfBrightnessX = 500
    region1.centerOfBrightnessY = 380
    region1.centerX = 500
    region1.centerY = 380
    region2 = messaging.RegionOfInterestMsgPayload()
    region2.numberOfPixels = 200
    region2.centerOfBrightnessX = 100
    region2.centerOfBrightnessY = 100
    region2.centerX = 100
    region2.centerY = 100
    region3 = messaging.RegionOfInterestMsgPayload()

    regions_msg_payload.regions = [region1, region2, region3]

    regions_input_msg = messaging.RegionsIdentifiedMsg().write(regions_msg_payload)
    roi_module.roisInMsg.subscribeTo(regions_input_msg)

    roi_log = roi_module.regionOutMsg.recorder()
    sim.AddModelToTask(task_name, roi_log)

    sim.InitializeSimulation()
    sim.ConfigureStopTime(macros.sec2nano(1.0))
    sim.ExecuteSimulation()

    # Verify only the windowed region was detected
    output_pixels = roi_log.numberOfPixels[-1]
    output_center_x = roi_log.centerOfBrightnessX[-1]

    np.testing.assert_allclose(output_pixels, 100, rtol=0, atol=PIXEL_COUNT_TOLERANCE,
                               err_msg="Should only detect windowed region")
    np.testing.assert_allclose(output_center_x, 500, rtol=0, atol=POSITION_TOLERANCE,
                               err_msg="Should detect region inside window")


def region_identification(show_plots, num_regions, use_windowing):
    """Main test function for region identification"""

    task_name = "unitTask"
    process_name = "TestProcess"
    log_rate = 0.1

    # Create simulation
    sim = SimulationBaseClass.SimBaseClass()
    test_process = sim.CreateNewProcess(process_name)
    test_process.addTask(sim.CreateNewTask(task_name, macros.sec2nano(log_rate)))

    # Create module
    roi_module = regionsOfInterest.RegionsOfInterest()
    roi_module.modelTag = "roi_module"
    sim.AddModelToTask(task_name, roi_module)

    # Configure module parameters
    roi_module.setMaxRoiSeparation(100)

    if use_windowing:
        window_center = [512, 384]
        roi_module.setWindowCenter(window_center)
        roi_module.setWindowSize(600, 400)

    # Create input regions
    regions_msg_payload = messaging.RegionsIdentifiedMsgPayload()

    # Define test regions with varying properties
    test_regions = [
        {'pixels': 150, 'x': 300, 'y': 300},
        {'pixels': 250, 'x': 512, 'y': 384},  # Center, brightest
        {'pixels': 120, 'x': 700, 'y': 500},
        {'pixels': 80, 'x': 200, 'y': 600},
        {'pixels': 180, 'x': 600, 'y': 300},
    ]

    region_list = []
    for i in range(min(num_regions, len(test_regions))):
        region = messaging.RegionOfInterestMsgPayload()
        region.numberOfPixels = test_regions[i]['pixels']
        region.centerOfBrightnessX =  test_regions[i]['x']
        region.centerOfBrightnessY =  test_regions[i]['y']
        region.centerX = test_regions[i]['x']
        region.centerY = test_regions[i]['y']
        region.width = 15
        region.height = 15
        region.timeTag = i * 0.1
        region_list.append(region)

    for i in range(num_regions, 3):
        region = messaging.RegionOfInterestMsgPayload()
        region_list.append(region)

    regions_msg_payload.regions = region_list
    regions_input_msg = messaging.RegionsIdentifiedMsg().write(regions_msg_payload)
    roi_module.roisInMsg.subscribeTo(regions_input_msg)

    # Set up logging
    roi_log = roi_module.regionOutMsg.recorder()
    sim.AddModelToTask(task_name, roi_log)

    # Run simulation
    sim_duration = 2.0
    sim.InitializeSimulation()
    sim.ConfigureStopTime(macros.sec2nano(sim_duration))
    sim.ExecuteSimulation()

    # Extract results
    output_pixels = roi_log.numberOfPixels
    output_center_x = roi_log.centerOfBrightnessX
    output_center_y = roi_log.centerOfBrightnessY
    output_width = roi_log.width
    output_height = roi_log.height

    # Verify results
    if num_regions > 0:
        # Should have detected something
        assert output_pixels[-1] > 0, "Should have detected regions"

        if use_windowing:
            # With windowing, should prefer the centered region
            np.testing.assert_allclose(output_center_x[-1], 512, rtol=0, atol=50,
                                       err_msg="Windowing should prefer centered region")
        else:
            regions = test_regions[:num_regions]
            max_pixels = max(regions, key=lambda x: x['pixels'])["pixels"]
            # Without windowing, should select brightest (250 pixels at 512, 384)
            np.testing.assert_allclose(output_pixels[-1], max_pixels, rtol=0, atol=PIXEL_COUNT_TOLERANCE,
                                       err_msg="Should select brightest region")

    # Plotting
    if show_plots:
        plt.close("all")

        # Plot detected region properties over time
        plt.figure(1, figsize=(12, 8))

        plt.subplot(2, 2, 1)
        plt.plot(roi_log.times() * macros.NANO2SEC, output_pixels, 'b-', linewidth=2)
        plt.xlabel('Time [s]')
        plt.ylabel('Pixel Count')
        plt.title('Detected Region Pixel Count')
        plt.grid(True)

        plt.subplot(2, 2, 2)
        plt.plot(roi_log.times() * macros.NANO2SEC, output_center_x, 'r-', linewidth=2, label='X')
        plt.plot(roi_log.times() * macros.NANO2SEC, output_center_y, 'g-', linewidth=2, label='Y')
        plt.xlabel('Time [s]')
        plt.ylabel('Position [pixels]')
        plt.title('Center of Brightness Position')
        plt.legend()
        plt.grid(True)

        plt.subplot(2, 2, 3)
        plt.plot(roi_log.times() * macros.NANO2SEC, output_width, 'b-', linewidth=2, label='Width')
        plt.plot(roi_log.times() * macros.NANO2SEC, output_height, 'r-', linewidth=2, label='Height')
        plt.xlabel('Time [s]')
        plt.ylabel('Size [pixels]')
        plt.title('Region Size')
        plt.legend()
        plt.grid(True)

        plt.subplot(2, 2, 4)
        # Visualize the regions on a 2D plot
        for i in range(min(num_regions, len(test_regions))):
            plt.scatter(test_regions[i]['x'], test_regions[i]['y'],
                       s=test_regions[i]['pixels'], alpha=0.5,
                       label=f"Region {i+1}")

        plt.scatter(output_center_x[-1], output_center_y[-1],
                   s=200, c='red', marker='*', edgecolors='black', linewidths=2,
                   label='Identified ROI', zorder=10)

        if use_windowing:
            window_center = np.array([512, 384])
            window_width = 600
            window_height = 400
            rect_x = window_center[0] - window_width/2
            rect_y = window_center[1] - window_height/2
            from matplotlib.patches import Rectangle
            rect = Rectangle((rect_x, rect_y), window_width, window_height,
                           linewidth=2, edgecolor='blue', facecolor='none',
                           label='Window')
            plt.gca().add_patch(rect)

        plt.xlabel('X Position [pixels]')
        plt.ylabel('Y Position [pixels]')
        plt.title('Spatial Distribution of Regions')
        plt.legend()
        plt.grid(True)
        plt.axis('equal')

        plt.tight_layout()
        plt.show()
        plt.close('all')


if __name__ == "__main__":
    region_identification(True,
                         3,
                         False)
