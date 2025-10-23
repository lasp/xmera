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
from xmera.fswAlgorithms import mrpSteering  # import the module that is to be tested
from xmera.fswAlgorithms import rateServoFullNonlinear
from xmera.utilities import RigidBodyKinematics
from xmera.utilities import SimulationBaseClass
from xmera.utilities import macros

@pytest.mark.parametrize("K1", [0.15, 0])
@pytest.mark.parametrize("K3", [1.0, 0])
@pytest.mark.parametrize("omega_max", [1.5 * macros.D2R, 0.001])
@pytest.mark.parametrize("ignore_feed_forward", [True, False])

def test_mrp_steering_tracking_integrated(show_plots, K1, K3, omega_max, ignore_feed_forward):
    unit_task_name = "unitTask"
    unit_process_name = "TestProcess"

    # Create a sim module as an empty container
    unit_test_sim = SimulationBaseClass.SimBaseClass()

    # Create test thread
    test_process_rate = macros.sec2nano(0.5)  # update process rate update time
    test_proc = unit_test_sim.CreateNewProcess(unit_process_name)
    test_proc.addTask(unit_test_sim.CreateNewTask(unit_task_name, test_process_rate))

    module = mrpSteering.MrpSteering()
    module.modelTag = "mrpSteering"

    servo = rateServoFullNonlinear.RateServoFullNonlinear()
    servo.modelTag = "rate_servo"

    unit_test_sim.AddModelToTask(unit_task_name, module)
    unit_test_sim.AddModelToTask(unit_task_name, servo)

    module.setK1(K1)
    module.setK3(K3)
    module.setOmegaMax(omega_max)
    module.setIgnoreFeedforward(ignore_feed_forward)

    servo.setKi(0.01)
    servo.setP(150.0)
    servo.setIntegralLimit(2. / servo.getKi() * 0.1)
    servo.setKnownTorquePntB_B([0., 0., 0.])

    # attGuidOut Message:
    guid_cmd_data = messaging.AttGuidMsgPayload()  # Create a structure for the input message
    guid_cmd_data.sigma_BR = [0.3, -0.5, 0.7]
    guid_cmd_data.omega_BR_B = [0.010, -0.020, 0.015]
    guid_cmd_data.omega_RN_B = [-0.02, -0.01, 0.005]
    guid_cmd_data.domega_RN_B = [0.0002, 0.0003, 0.0001]
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
    omega = [10.0, 25.0, 50.0, 100.0]
    rw_speed_message.wheelSpeeds = omega
    rw_in_msg = messaging.RWSpeedMsg().write(rw_speed_message)

    # wheelConfigData message
    rw_config_params = messaging.RWArrayConfigMsgPayload()
    rw_config_params.GsMatrix_B = [
        1.0, 0.0, 0.0,
        0.0, 1.0, 0.0,
        0.0, 0.0, 1.0,
        0.5773502691896258, 0.5773502691896258, 0.5773502691896258
    ]
    rw_config_params.JsList = [0.1, 0.1, 0.1, 0.1]
    rw_config_params.numRW = 4
    rw_param_in_msg = messaging.RWArrayConfigMsg().write(rw_config_params)

    # wheelAvailability message
    rw_avail_list = []
    rw_availability_message = messaging.RWAvailabilityMsgPayload()
    rw_avail = [messaging.AVAILABLE, messaging.AVAILABLE, messaging.AVAILABLE, messaging.AVAILABLE]
    rw_availability_message.wheelAvailability = rw_avail
    rw_avail_in_msg = messaging.RWAvailabilityMsg().write(rw_availability_message)
    rw_avail_list.append(rw_avail)

    # Setup logging on the test module output message so that we get all the writes to it
    data_log = servo.cmdTorqueOutMsg.recorder()
    unit_test_sim.AddModelToTask(unit_task_name, data_log)

    # connect messages
    module.guidInMsg.subscribeTo(guid_in_msg)
    servo.guidInMsg.subscribeTo(guid_in_msg)
    servo.vehConfigInMsg.subscribeTo(vc_in_msg)
    servo.rwParamsInMsg.subscribeTo(rw_param_in_msg)
    servo.vehConfigInMsg.subscribeTo(vc_in_msg)
    servo.rwSpeedsInMsg.subscribeTo(rw_in_msg)
    servo.rateSteeringInMsg.subscribeTo(module.rateCmdOutMsg)
    servo.rwAvailInMsg.subscribeTo(rw_avail_in_msg)

    unit_test_sim.InitializeSimulation()
    unit_test_sim.ConfigureStopTime(macros.sec2nano(1.0))  # seconds to stop simulation
    unit_test_sim.ExecuteSimulation()

    servo.reset(1)  # this module reset function needs a time input (in NanoSeconds)

    unit_test_sim.ConfigureStopTime(macros.sec2nano(2.0))  # seconds to stop simulation
    unit_test_sim.ExecuteSimulation()

    # Compute true values
    true_vals = find_true_torques(module, servo, guid_cmd_data, rw_speed_message, vehicle_config_out, rw_avail_list, rw_config_params)

    # compare the module results to the truth values
    accuracy = 1e-12

    np.testing.assert_allclose(data_log.torqueRequestBody, true_vals, atol=accuracy, rtol=0, verbose=True)


def find_true_values(guid_cmd_data, module):

    omega_max = module.getOmegaMax()
    sigma = np.asarray(guid_cmd_data.sigma_BR)
    K1 = np.asarray(module.getK1())
    K3 = np.asarray(module.getK3())
    B = RigidBodyKinematics.BmatMRP(sigma)
    omega_ast = []
    omega_ast_p = []

    for i in range(len(sigma)):
        steer_rate = -1*(2*omega_max/np.pi)*np.arctan((K1*sigma[i]+K3*sigma[i]*sigma[i]*sigma[i])*np.pi/(2*omega_max))
        omega_ast.append(steer_rate)

    if not module.getIgnoreFeedforward():
        sigma_p = 0.25*B.dot(omega_ast)
        for i in range(len(sigma)):
            omega_ast_rate = (K1+3*K3*sigma[i]**2)/(1+((K1*sigma[i]+K3*sigma[i]**3)**2)*(np.pi/(2*omega_max))**2)*sigma_p[i]
            omega_ast_p.append(-omega_ast_rate)
    else:
        omega_ast_p = np.asarray([0, 0, 0])

    return omega_ast, omega_ast_p

def find_true_torques(module, servo, guid_cmd_data, rw_speed_message, vehicle_config_out, rw_avail_msg, rw_config_params):
    Lr = []

    #Read in variables
    num_rw = rw_config_params.numRW
    L = np.asarray(servo.getKnownTorquePntB_B()).flatten()
    steps = [0, 0, .5, 0, .5]
    omega_BR_B = np.asarray(guid_cmd_data.omega_BR_B)
    omega_RN_B = np.asarray(guid_cmd_data.omega_RN_B)
    omega_BN_B = omega_BR_B + omega_RN_B #find body rate
    domega_RN_B = np.asarray(guid_cmd_data.domega_RN_B)

    omega_BastR_B, omegap_BastR_B = find_true_values(guid_cmd_data, module)

    omega_BastN_B = omega_BastR_B+omega_RN_B
    omega_BBast_B = omega_BN_B - omega_BastN_B

    Isc = np.asarray(vehicle_config_out.ISCPntB_B)
    Isc = np.reshape(Isc, (3, 3))
    Ki = servo.getKi()
    P = servo.getP()
    jsVec = rw_config_params.JsList[0:num_rw]
    GsMatrix = (rw_config_params.GsMatrix_B)
    GsMatrix_B_array = np.reshape(GsMatrix[0:num_rw * 3], (num_rw, 3))

    # Compute toruqes
    for i in range(len(steps)):
        dt = steps[i]
        if dt == 0:
            zVec = np.asarray([0, 0, 0])

        #evaluate integral term
        if Ki > 0 and abs(servo.getIntegralLimit()) > 0: #if integral feedback is on
            zVec = dt * omega_BBast_B + zVec  # z = integral(del_omega)
            # Make sure each component is less than the integral limit
            for i in range(3):
                if zVec[i] > servo.getIntegralLimit():
                        zVec[i] = zVec[i]/abs(zVec[i])*servo.getIntegralLimit()

        else: # integral gain turned off/negative setting
            zVec = np.asarray([0, 0, 0])

        #compute torque Lr
        Lr0 = Ki * zVec  # +K*sigmaBR
        Lr1 = Lr0 + P * omega_BBast_B  # +P*deltaOmega

        GsHs = np.array([0,0,0])

        if num_rw > 0:
            for i in range(num_rw):
                if rw_avail_msg[0][i] == 0:  # Make RW availability check
                    GsHs = GsHs + np.dot(GsMatrix_B_array[i, :], jsVec[i] * (np.dot(omega_BN_B, GsMatrix_B_array[i, :]) + rw_speed_message.wheelSpeeds[i]))
                    # J_s*(dot(omegaBN_B,Gs_vec)+Omega_wheel)

        Lr2 = Lr1 - np.cross(omega_BastN_B, (Isc.dot(omega_BN_B)+GsHs))  #  - omega_BastN x ([I]omega + [Gs]h_s)

        Lr3 = Lr2 - Isc.dot(omegap_BastR_B + domega_RN_B - np.cross(omega_BN_B, omega_RN_B))
        # - [I](d(omega_B^ast/R)/dt + d(omega_r)/dt - omega x omega_r)
        Lr4 = Lr3 + L
        Lr4 = -Lr4
        Lr.append(np.ndarray.tolist(Lr4))

    return Lr

if __name__ == "__main__":
    test_mrp_steering_tracking_integrated(False, 0.15, 1.0, 0.025, False)
