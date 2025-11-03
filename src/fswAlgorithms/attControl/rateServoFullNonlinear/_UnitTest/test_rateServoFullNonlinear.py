#
#  ISC License
#
#  Copyright (c) 2025, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder
#
#  Permission to use, copy, modify, and/or distribute this software for any
#  purpose with or without fee is hereby granted, provided that the above
#  copyright notice and this permission notice appear in all copies.
#
#  THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
#  WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
#  MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
#  ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
#  WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
#  ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
#  OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
#

import numpy as np
import pytest
from xmera.architecture import messaging
from xmera.fswAlgorithms import rateServoFullNonlinear  # import the module that is to be tested
from xmera.utilities import SimulationBaseClass
from xmera.utilities import macros

@pytest.mark.parametrize("rw_num", [4, 0])
@pytest.mark.parametrize("int_gain", [0.01, -1])
@pytest.mark.parametrize("omegap_BastR_B", [[1.87766650e-04, -3.91233583e-05, 3.56369489e-05], [0, 0, 0]])
@pytest.mark.parametrize("omega_BastR_B",  [[-2.23886891e-02, 2.47942516e-02, -2.55601849e-02], [0, 0, 0]])
@pytest.mark.parametrize("integral_limit", [0, 20])
@pytest.mark.parametrize("use_rw_availability", ["NO", "ON", "OFF"])

def test_rate_servo_full_nonlinear(show_plots, rw_num, int_gain, omegap_BastR_B, omega_BastR_B, integral_limit,
                                   use_rw_availability):
    unit_task_name = "unitTask"  # arbitrary name (don't change)
    unit_process_name = "TestProcess"  # arbitrary name (don't change)

    #   Create a sim module as an empty container
    unit_test_sim = SimulationBaseClass.SimBaseClass()

    # Create test thread
    test_process_rate = macros.sec2nano(0.5)  # update process rate update time
    test_proc = unit_test_sim.CreateNewProcess(unit_process_name)
    test_proc.addTask(unit_test_sim.CreateNewTask(unit_task_name, test_process_rate))

    # Construct algorithm and associated C++ container
    module = rateServoFullNonlinear.RateServoFullNonlinear()
    module.modelTag = "rate_servo"

    # Add test module to runtime call list
    unit_test_sim.AddModelToTask(unit_task_name, module)

    # configure module parameters
    module.setKi(int_gain)
    module.setP(150.0)
    module.setIntegralLimit(integral_limit)
    module.setKnownTorquePntB_B([1,1,1])

    #   Create input message and size it because the regular creator of that message
    #   is not part of the test.
    #   attGuidOut Message:
    guid_cmd_data = messaging.AttGuidMsgPayload()  # Create a structure for the input message
    sigma_BR = np.array([0.3, -0.5, 0.7])
    guid_cmd_data.sigma_BR = sigma_BR
    omega_BR_B = np.array([0.010, -0.020, 0.015])
    guid_cmd_data.omega_BR_B = omega_BR_B
    omega_RN_B = np.array([-0.02, -0.01, 0.005])
    guid_cmd_data.omega_RN_B = omega_RN_B
    domega_RN_B = np.array([0.0002, 0.0003, 0.0001])
    guid_cmd_data.domega_RN_B = domega_RN_B
    guid_in_msg = messaging.AttGuidMsg().write(guid_cmd_data)

    # vehicleConfigData Message:
    vehicle_config_out = messaging.VehicleConfigMsgPayload()
    I = [1000., 0., 0.,
         0., 800., 0.,
         0., 0., 800.]
    vehicle_config_out.ISCPntB_B = I
    vc_in_msg = messaging.VehicleConfigMsg().write(vehicle_config_out)

    # wheelSpeeds Message
    rw_speed_message = messaging.RWSpeedMsgPayload()
    Omega = [10.0, 25.0, 50.0, 100.0]
    rw_speed_message.wheelSpeeds = Omega
    rw_speed_in_msg = messaging.RWSpeedMsg().write(rw_speed_message)

    # wheelConfigData message
    rw_config_params = messaging.RWArrayConfigMsgPayload()
    js_list = [0.1, 0.1, 0.1, 0.1]
    G_s_B = [
        1.0, 0.0, 0.0,
        0.0, 1.0, 0.0,
        0.0, 0.0, 1.0,
        0.5773502691896258, 0.5773502691896258, 0.5773502691896258
    ]
    rw_config_params.GsMatrix_B = G_s_B
    rw_config_params.JsList = js_list
    rw_config_params.numRW = rw_num
    rw_param_in_msg = messaging.RWArrayConfigMsg().write((rw_config_params))

    # wheelAvailability message
    rw_availability_message = messaging.RWAvailabilityMsgPayload()
    if use_rw_availability != "NO":
        if use_rw_availability == "ON":
            rw_availability_message.wheelAvailability  = [messaging.AVAILABLE, messaging.AVAILABLE,
                                                        messaging.AVAILABLE, messaging.AVAILABLE]
        elif use_rw_availability == "OFF":
            rw_availability_message.wheelAvailability  = [messaging.UNAVAILABLE, messaging.UNAVAILABLE,
                                                        messaging.UNAVAILABLE, messaging.UNAVAILABLE]
        else:
            print("WARNING: unknown rw availability status")
        rw_avail_in_msg = messaging.RWAvailabilityMsg().write(rw_availability_message)
    else:
        # set default availability
        rw_availability_message.wheelAvailability = [messaging.AVAILABLE, messaging.AVAILABLE,
                                                   messaging.AVAILABLE, messaging.AVAILABLE]

    # rateSteering message
    rate_steering_msg = messaging.RateCmdMsgPayload()
    rate_steering_msg.omega_BastR_B = omega_BastR_B
    rate_steering_msg.omegap_BastR_B = omegap_BastR_B
    rate_cmd_in_msg = messaging.RateCmdMsg().write(rate_steering_msg)

    # Setup logging on the test module output message so that we get all the writes to it
    data_log = module.cmdTorqueOutMsg.recorder()
    unit_test_sim.AddModelToTask(unit_task_name, data_log)

    # Initialize the test module configuration data
    module.guidInMsg.subscribeTo(guid_in_msg)
    module.vehConfigInMsg.subscribeTo(vc_in_msg)
    module.rwParamsInMsg.subscribeTo(rw_param_in_msg)
    module.vehConfigInMsg.subscribeTo(vc_in_msg)
    module.rwSpeedsInMsg.subscribeTo(rw_speed_in_msg)
    module.rateSteeringInMsg.subscribeTo(rate_cmd_in_msg)
    if use_rw_availability != "NO":
        module.rwAvailInMsg.subscribeTo(rw_avail_in_msg)

    # Need to call the self-init and cross-init methods
    unit_test_sim.InitializeSimulation()

    # Step the simulation to 3*process rate so 4 total steps including zero
    unit_test_sim.ConfigureStopTime(macros.sec2nano(1.0))  # seconds to stop simulation
    unit_test_sim.ExecuteSimulation()

    module.reset(1)  # this module reset function needs a time input (in NanoSeconds)

    unit_test_sim.ConfigureStopTime(macros.sec2nano(2.0))  # seconds to stop simulation
    unit_test_sim.ExecuteSimulation()

    # set the filtered output truth states
    Lr_true = find_true_torques(module, guid_cmd_data, rw_speed_message, vehicle_config_out, js_list,
                                rw_num, G_s_B, rw_availability_message, rate_steering_msg)

    Lr = data_log.torqueRequestBody

    # compare the module results to the truth values
    accuracy = 1e-8
    np.testing.assert_allclose(Lr, Lr_true, atol=accuracy, rtol=0, verbose=True)


def find_true_torques(module, guid_cmd_data, rw_speed_message, vehicle_config_out, js_list, num_rw, G_s_B, rw_avail_msg, rate_steering_msg):
    Lr = []

    #Read in variables
    L = np.asarray(module.getKnownTorquePntB_B()).flatten()
    steps = [0, 0, .5, 0, .5]
    omega_BR_B = np.asarray(guid_cmd_data.omega_BR_B)
    omega_RN_B = np.asarray(guid_cmd_data.omega_RN_B)
    omega_BN_B = omega_BR_B + omega_RN_B #find body rate
    domega_RN_B = np.asarray(guid_cmd_data.domega_RN_B)
    omega_BastR_B = np.asarray(rate_steering_msg.omega_BastR_B)
    omegap_BastR_B = np.asarray(rate_steering_msg.omegap_BastR_B) #body-frame derivative of omega_BastR_B
    omega_BastN_B = omega_BastR_B+omega_RN_B
    omega_BBast_B = omega_BN_B - omega_BastN_B

    Isc = np.asarray(vehicle_config_out.ISCPntB_B)
    Isc = np.reshape(Isc, (3, 3))
    Ki = module.getKi()
    P = module.getP()
    js_vec = js_list
    G_s_B_array = np.asarray(G_s_B)
    G_s_B_array = np.reshape(G_s_B_array[0:num_rw * 3], (num_rw, 3))

    #Compute toruqes
    for i in range(len(steps)):
        dt = steps[i]
        if dt == 0:
            z_vec = np.asarray([0, 0, 0])

        #evaluate integral term
        if Ki > 0 and abs(module.getIntegralLimit()) > 0: #if integral feedback is on
            z_vec = dt * omega_BBast_B + z_vec  # z = integral(del_omega)
            # Make sure each component is less than the integral limit
            for i in range(3):
                if z_vec[i] > module.getIntegralLimit():
                        z_vec[i] = z_vec[i]/abs(z_vec[i])*module.getIntegralLimit()

        else: #integral gain turned off/negative setting
            z_vec = np.asarray([0, 0, 0])

        #compute torque Lr
        Lr0 = Ki * z_vec  # +K*sigmaBR
        Lr1 = Lr0 + P * omega_BBast_B  # +P*deltaOmega

        GsHs = np.array([0,0,0])

        if num_rw > 0:
            for i in range(num_rw):
                if rw_avail_msg.wheelAvailability[i] == 0:  # Make RW availability check
                    GsHs = GsHs + np.dot(G_s_B_array[i, :], js_vec[i] * (np.dot(omega_BN_B, G_s_B_array[i, :]) + rw_speed_message.wheelSpeeds[i]))
                    # J_s*(dot(omegaBN_B,Gs_vec)+Omega_wheel)

        Lr2 = Lr1 - np.cross(omega_BastN_B, (Isc.dot(omega_BN_B)+GsHs))  #  - omega_BastN x ([I]omega + [Gs]h_s)

        Lr3 = Lr2 - Isc.dot(omegap_BastR_B + domega_RN_B - np.cross(omega_BN_B, omega_RN_B))
        # - [I](d(omega_B^ast/R)/dt + d(omega_r)/dt - omega x omega_r)
        Lr4 = Lr3 + L
        Lr4 = -Lr4
        Lr.append(np.ndarray.tolist(Lr4))
    return np.array(Lr)


if __name__ == "__main__":
    test_rate_servo_full_nonlinear(False, #show plots T/F
                                   4,           # Number of RW
                                   0.01,        # Integral Gain
                                   [0, 0, 0],   # omegap_BastR_B
                                   [0, 0, 0],   # omega_BastR_B
                                   20,          # integraLimit
                                   "ON")        # useRwAvailability
