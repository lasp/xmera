# SPDX-License-Identifier: ISC
# Copyright (c) 2016, Autonomous Vehicle System Lab, University of Colorado at Boulder
# Copyright (c) 2025, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder
#

import numpy as np
import pytest
from xmera.architecture import messaging
from xmera.simulation import starTracker
from xmera.utilities import RigidBodyKinematics as rbk
from xmera.utilities import SimulationBaseClass
from xmera.utilities import macros

def list_stack(vec, sim_stop_time, unit_proc_rate):
    # returns a list duplicated the number of times needed to be consistent with module output
    return [vec] * int(sim_stop_time / (float(unit_proc_rate) / float(macros.sec2nano(1))))

def set_random_walk(self, sen_noise_std = 0.0, error_bounds = [[1e6], [1e6], [1e6]]):
    # sets the module random walk variables
    p_matrix = [[sen_noise_std, 0., 0.], [0., sen_noise_std, 0.], [0., 0., sen_noise_std]]
    self.setPMatrix(p_matrix)
    self.setWalkBounds(error_bounds)

# uncomment this line is this test is to be skipped in the global unit test run, adjust message as needed
# @pytest.mark.skipif(conditionstring)
# uncomment this line if this test has an expected failure, adjust message as needed
@pytest.mark.parametrize("use_flag, test_case", [
    (False, 'basic'),
    (False, 'noise'),
    (False, 'walk bounds'),
    (False, 'angular velocity check')
])
def test_starTracker(show_plots, use_flag, test_case):
    unit_task_name = "unitTask"
    unit_proc_name = "TestProcess"

    # Create a sim module as an empty container
    unit_sim = SimulationBaseClass.SimBaseClass()

    unit_proc_rate = macros.sec2nano(0.1)
    unit_proc_rate_s = macros.NANO2SEC*unit_proc_rate
    unit_proc = unit_sim.CreateNewProcess(unit_proc_name)
    unit_proc.addTask(unit_task_name, unit_proc_rate)

    # Configure the starTracker module
    str_tracker = starTracker.StarTracker()
    str_tracker.modelTag = "starTracker"
    set_random_walk(str_tracker)
    unit_sim.AddModelToTask(unit_task_name, str_tracker)

    # Configure starTracker SCState input message
    sc_states_message_data = messaging.SCStatesMsgPayload()
    sc_states_message_data.r_BN_N = [0, 0, 0]
    sc_states_message_data.v_BN_N = [0, 0, 0]
    sc_states_message_data.sigma_BN = [0, 0, 0]
    sc_states_message_data.omega_BN_B = [0, 0, 0]
    sc_states_message_data.TotalAccumDVBdy = [0, 0, 0]
    sc_states_message_data.MRPSwitchCount = 0

    true_vector = dict()

    # This test verifies basic input and output
    if test_case == 'basic':
        sim_stop_time = 0.5
        prv_cb = [0.0, 0.0, 10.0 * macros.D2R]
        dcm_cb = rbk.PRV2C(prv_cb)
        str_tracker.setDcmCB(dcm_cb)
        sigma_bn = np.array([-0.390614710591786, -0.503642740963740, 0.462959869561285])
        sigma_cb = rbk.C2MRP(dcm_cb)
        sigma_cn = rbk.addMRP(sigma_bn, sigma_cb)
        beta_cn = rbk.MRP2EP(sigma_cn)
        sc_states_message_data.sigma_BN = sigma_bn
        true_vector['qInrtl2Case'] = list_stack(beta_cn, sim_stop_time, unit_proc_rate)
        true_vector['timeTag'] = np.arange(0, 0 + sim_stop_time*1E9, unit_proc_rate_s*1E9)

    elif test_case == 'noise':
        sim_stop_time = 1000.
        noise_std = 0.1
        std_correction_factor = 1.5  # This needs to be used because of the Gauss Markov module. need to fix the GM module
        set_random_walk(str_tracker, noise_std * std_correction_factor, [[1.0e-13], [1.0e-13], [1.0e-13]])
        sigma_bn = np.array([0, 0, 0])
        sc_states_message_data.sigma_BN = sigma_bn
        true_vector['qInrtl2Case'] = [noise_std] * 3
        true_vector['timeTag'] = np.arange(0, 0 + sim_stop_time*1E9, unit_proc_rate_s*1E9)

    # This test checks the walk bounds of random walk
    elif test_case == 'walk bounds':
        sim_stop_time = 1000.
        noise_std = 0.01
        std_correction_factor = 1.5  # This needs to be used because of the Gauss Markov module. need to fix the GM module
        walk_bound = 0.1
        set_random_walk(str_tracker, noise_std * std_correction_factor, [[walk_bound], [walk_bound], [walk_bound]])
        sigma_bn = np.array([0, 0, 0])
        sc_states_message_data.sigma_BN = sigma_bn
        true_vector['qInrtl2Case'] = [walk_bound + noise_std*3] * 3
        true_vector['timeTag'] = np.arange(0, 0+sim_stop_time*1E9, unit_proc_rate_s*1E9)

    # This test checks the computed platform rate
    elif test_case == 'angular velocity check':
        prv_cb = [10.0 * macros.D2R, 0.0, 0.0]
        dcm_cb = rbk.PRV2C(prv_cb)
        str_tracker.setDcmCB(dcm_cb)
        sim_stop_time = unit_proc_rate_s
        sigma_bn = np.array([0, 0, 0])
        sc_states_message_data.sigma_BN = sigma_bn
    else:
        raise Exception('invalid test case')

    # Set up data logging
    star_tracker_sensor_msg_data_log = str_tracker.sensorOutMsg.recorder()
    unit_sim.AddModelToTask(unit_task_name, star_tracker_sensor_msg_data_log)

    # Configure spacecraft state message
    sc_states_message = messaging.SCStatesMsg().write(sc_states_message_data)
    str_tracker.scStateInMsg.subscribeTo(sc_states_message)

    unit_sim.InitializeSimulation()
    unit_sim.ConfigureStopTime(macros.sec2nano(sim_stop_time))
    unit_sim.ExecuteSimulation()

    # Run additional simulation chunks for angular velocity test check
    sigma_bn_list = [np.array([0.0, 0.0, 0.0])]
    if test_case == 'angular velocity check':
        rot_axis_n = np.array([1.0, 0.0, 0.0])  # Hub rotation axis
        prv_angle_list = np.array([0.0, 169.0, 169.9, 170.1, 171.2]) * macros.D2R  # Truth hub attitudes

        for idx in range(1, len(prv_angle_list)):
            # Compute hub inertial attitude
            prv_bn = prv_angle_list[idx] * rot_axis_n
            sigma_bn = np.array(rbk.PRV2MRP(prv_bn))
            sigma_bn_list.append(sigma_bn)

            # Update and connect the sc state message to the star tracker module
            sc_states_message_data = messaging.SCStatesMsgPayload()
            sc_states_message_data.r_BN_N = [0, 0, 0]
            sc_states_message_data.v_BN_N = [0, 0, 0]
            sc_states_message_data.sigma_BN = sigma_bn
            sc_states_message_data.omega_BN_B = [0, 0, 0]
            sc_states_message_data.TotalAccumDVBdy = [0, 0, 0]
            sc_states_message_data.MRPSwitchCount = 0
            sc_states_message = messaging.SCStatesMsg().write(sc_states_message_data)
            str_tracker.scStateInMsg.subscribeTo(sc_states_message)

            # Execute simulation chunk for updated spacecraft attitude
            sim_stop_time = sim_stop_time + unit_proc_rate_s
            unit_sim.ConfigureStopTime(macros.sec2nano(sim_stop_time))
            unit_sim.ExecuteSimulation()

        sigma_bn_list = np.vstack(sigma_bn_list)

    # Extract logged data for test check
    timespan = macros.NANO2SEC * star_tracker_sensor_msg_data_log.times()  # [s]
    beta_cn = star_tracker_sensor_msg_data_log.qInrtl2Case
    omega_cn_c_module = macros.R2D * star_tracker_sensor_msg_data_log.omega_CN_C  # [rad/s]

    # Convert quaternion output to prv
    prv_cn = np.zeros([int(sim_stop_time/unit_proc_rate_s)+1, 3])
    for i in range(0, int(sim_stop_time/unit_proc_rate_s)+1):
        if not np.allclose(beta_cn[i], [1.0, 0.0, 0.0, 0.0]):
            prv_cn[i] = rbk.EP2PRV(beta_cn[i])
        else:
            prv_cn[i] = [0.0, 0.0, 0.0]

    accuracy = 1e-6
    if test_case == 'noise':
        bound_array = np.full((int(sim_stop_time/unit_proc_rate_s)+1), 0.01)
        for i in range(0, 3):
            np.testing.assert_array_less(np.abs(np.mean(prv_cn[:, i])),
                                         bound_array,
                                         verbose=True)

            np.testing.assert_array_less(np.abs(np.std(prv_cn[:, i]) - true_vector['qInrtl2Case'][i]),
                                         bound_array,
                                         verbose=True)

    elif test_case == 'walk bounds':
        for i in range(0, 3):
            np.testing.assert_array_less(np.max(np.abs(np.asarray(prv_cn[i]))),
                                         true_vector['qInrtl2Case'][i],
                                         verbose=True)

    elif test_case == 'angular velocity check':
        # Check computed platform angular velocity
        omega_cn_c_truth = [np.zeros((1, 3))]
        current_sim_time = timespan[0]
        for idx in range(1, len(timespan)):
            # Grab the current and previous quaternions for numerical differentiation
            beta_current_cn = np.array(beta_cn[idx, :])
            beta_previous_cn = np.array(beta_cn[idx-1, :])

            # Compute beta_dot_cn
            previous_sim_time = current_sim_time
            current_sim_time = timespan[idx]
            dt = current_sim_time - previous_sim_time
            beta_dot_cn = (beta_current_cn - beta_previous_cn) / dt

            # Solve for platform rate using Eq. 3.106 from Schaub and Junkins 3rd edition Pg 111
            b_inv = rbk.BinvEP(beta_current_cn)

            omega_cn_c = 2 * b_inv.dot(beta_dot_cn)
            omega_cn_c_truth.append(macros.R2D * omega_cn_c)  # [deg]

        omega_cn_c_truth = np.vstack(omega_cn_c_truth)

        np.testing.assert_allclose(omega_cn_c_module,
                                   omega_cn_c_truth,
                                   atol=accuracy,
                                   verbose=True)

    else:
        for i in range(0, len(true_vector['qInrtl2Case'])):
            np.testing.assert_allclose(beta_cn[i],
                                       true_vector['qInrtl2Case'][i],
                                       atol=accuracy,
                                       verbose=True)

if __name__ == "__main__":
    test_starTracker(
        False,  # show_plots
        False,  # useFlag
        'angular velocity check'  # testCase
    )
