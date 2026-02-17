# SPDX-License-Identifier: ISC
# Copyright (c) 2025, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder
#

import os

import numpy as np
import pytest
from xmera import __path__
from xmera.architecture import messaging
from xmera.fswAlgorithms import timeClosestApproach
from xmera.utilities import SimulationBaseClass, macros

bskPath = __path__[0]
fileName = os.path.basename(os.path.splitext(__file__)[0])


@pytest.mark.parametrize("position", [[-5e7, 7.5e6, 5e5], [-5e6, 7e6, 4e5]])  # m
@pytest.mark.parametrize("velocity", [[2e4, 0, 0], [1e4, 1e3, 2e2]])  # m/s
@pytest.mark.parametrize("filter_covariance", [np.eye(6),  np.ones([6, 6]), np.eye(3), np.ones([3, 3])])

def test_time_closest_approach(show_plots, position, velocity, filter_covariance):

    unit_task_name = "unitTask"               # arbitrary name (don't change)
    unit_process_name = "test_processes"         # arbitrary name (don't change)

    # Create a sim module as an empty container
    unit_test_sim = SimulationBaseClass.SimBaseClass()

    # Create test thread
    test_process_rate = macros.sec2nano(0.1)     # update process rate update time
    test_process = unit_test_sim.CreateNewProcess(unit_process_name)
    test_process.addTask(unit_test_sim.CreateNewTask(unit_task_name, test_process_rate))

    # setup TimeClosestApproach guidance module
    tca_module = timeClosestApproach.TimeClosestApproach()
    tca_module.modelTag = "TimeClosestApproach"
    unit_test_sim.AddModelToTask(unit_task_name, tca_module)

    state_vector = np.zeros(6)
    state_vector[:3] = position
    state_vector[3:] = velocity

    # Create the input messages.
    input_filter_data = messaging.FilterMsgPayload()
    input_nav_data = messaging.NavTransMsgPayload()
    input_nav_data.r_BN_N = state_vector.tolist()[0:3]
    input_nav_data.v_BN_N = state_vector.tolist()[3:6]
    input_filter_data.numberOfStates = len(filter_covariance[:,0])
    input_filter_data.covar = filter_covariance.flatten().tolist()
    filter_in_msg = messaging.FilterMsg().write(input_filter_data)
    nav_in_msg = messaging.NavTransMsg().write(input_nav_data)
    tca_module.filterInMsg.subscribeTo(filter_in_msg)
    tca_module.navFilterMsg.subscribeTo(nav_in_msg)

    # Output messages.
    data_log_tca = tca_module.tcaOutMsg.recorder()
    unit_test_sim.AddModelToTask(unit_task_name, data_log_tca)

    unit_test_sim.InitializeSimulation()
    unit_test_sim.ConfigureStopTime(test_process_rate)
    unit_test_sim.ExecuteSimulation()

    # tca_module outputs
    tca_tca = data_log_tca.timeClosestApproach
    sigmatca_tca = data_log_tca.standardDeviation

    # Expected TCA
    tca, tca_covariance = time_of_closest_approach_calculation(position, velocity, filter_covariance)

    # Expected TCA when velocity is doubled
    tca_tca_value = tca_tca[0]
    tca_doubled_v = tca_with_doubled_velocity(position, velocity, filter_covariance)


    # make sure module output data is correct
    tolerance = 1e-10
    np.testing.assert_allclose(tca_tca,
                               tca,
                               rtol=0,
                               atol=tolerance,
                               err_msg='Variable: tca',
                               verbose=True)

    np.testing.assert_allclose(sigmatca_tca,
                               tca_covariance,
                               rtol=0,
                               atol=tolerance,
                               err_msg='Variable: tca_covariance',
                               verbose=True)

    np.testing.assert_(tca_doubled_v < tca_tca_value, msg="TCA_2 is not smaller than TCA_1"
)


def time_of_closest_approach_calculation(r, v, filter_covariance):

    norm_v = np.linalg.norm(v)
    norm_r = np.linalg.norm(r)
    v_hat = v / norm_v
    r_hat = r / norm_r
    theta = np.arccos(np.dot(-r_hat, v_hat))
    flight_path_angle = theta - np.pi/2
    ratio = norm_v/ norm_r

    tca = np.cos(theta) / ratio
    state_size = len(filter_covariance[:,0])

    covariance_map_to_tca = np.zeros(state_size)
    covariance_map_to_tca[0:3] = v_hat/norm_r
    if state_size == 6:
        covariance_map_to_tca[3:6] = 1/norm_v * (r_hat - np.sin(flight_path_angle) * v_hat)
    tca_covariance = (1 / ratio**2) * np.dot(covariance_map_to_tca,  np.dot(filter_covariance, covariance_map_to_tca.transpose()))

    return tca, np.sqrt(tca_covariance)


def tca_with_doubled_velocity(position, velocity, filter_covariance):
    """Return tca_2 when the velocity is doubled."""

    velocity_2 = 2.0 * np.array(velocity)   # Double the input velocity
    tca_2, _ = time_of_closest_approach_calculation(
        np.array(position), velocity_2, np.array(filter_covariance)
    )
    return tca_2


if __name__ == "__main__":
    test_time_closest_approach(True,
                             np.array([-5e7, 7.5e6, 5e5]),
                             np.array([2e4, 0, 0]),
                             np.eye(3)
                             )
