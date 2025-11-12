import numpy as np
import pytest
from xmera.architecture import messaging
from xmera.fswAlgorithms import mrpFeedback
from xmera.utilities import SimulationBaseClass
from xmera.utilities import macros

@pytest.mark.parametrize("int_gain", [0.01, -1])
@pytest.mark.parametrize("rw_num", [4, 0])
@pytest.mark.parametrize("integral_limit", [0, 20])
@pytest.mark.parametrize("ctrl_law", [0, 1])
@pytest.mark.parametrize("use_rw_availability", ["NO", "ON", "OFF"])

def test_mrp_feedback(show_plots, int_gain, rw_num, integral_limit, ctrl_law, use_rw_availability):
    unit_task_name = "unitTask"               # arbitrary name (don't change)
    unit_process_name = "TestProcess"         # arbitrary name (don't change)

    #   Create a sim module as an empty container
    unit_test_sim = SimulationBaseClass.SimBaseClass()

    #   Create test thread
    test_process_rate = macros.sec2nano(0.5)     # update process rate update time
    test_proc = unit_test_sim.CreateNewProcess(unit_process_name)
    test_proc.addTask(unit_test_sim.CreateNewTask(unit_task_name, test_process_rate))

    #   Construct algorithm and associated C++ container
    module = mrpFeedback.MrpFeedback()
    module.modelTag = "mrpFeedback"

    #   Add test module to runtime call list
    unit_test_sim.AddModelToTask(unit_task_name, module)

    #   Initialize the test module configuration data
    module.setK(0.15)
    module.setKi(int_gain)
    module.setP(150.0)
    module.setIntegralLimit(integral_limit)
    module.setControlLawType(ctrl_law)
    module.setKnownTorquePntB_B([1., 1., 1.])

    # create input messages
    #   AttGuidFswMsg Message:
    guid_cmd_data = messaging.AttGuidMsgPayload()
    sigma_BR = [0.3, -0.5, 0.7]
    guid_cmd_data.sigma_BR = sigma_BR
    omega_BR_B = [0.010, -0.020, 0.015]
    guid_cmd_data.omega_BR_B = omega_BR_B
    omega_RN_B = [-0.02, -0.01, 0.005]
    guid_cmd_data.omega_RN_B = omega_RN_B
    domega_RN_B = [0.0002, 0.0003, 0.0001]
    guid_cmd_data.domega_RN_B = domega_RN_B
    guid_in_msg = messaging.AttGuidMsg().write(guid_cmd_data)

    # vehicleConfigData Message:
    vehicle_config = messaging.VehicleConfigMsgPayload()
    I = [1000., 0., 0.,
         0., 800., 0.,
         0., 0., 800.]
    vehicle_config.ISCPntB_B = I
    vc_in_msg = messaging.VehicleConfigMsg().write(vehicle_config)

    # wheelSpeeds Message
    rw_speed_message = messaging.RWSpeedMsgPayload()
    Omega = [10.0, 25.0, 50.0, 100.0]  # rad/sec
    rw_speed_message.wheelSpeeds = Omega
    rw_speed_in_msg = messaging.RWSpeedMsg().write(rw_speed_message)

    # wheelConfigData message
    js_list = []
    G_s_B = []
    if rw_num > 0:
        rw_config_params = messaging.RWArrayConfigMsgPayload()

        G_s_B = [
            1.0, 0.0, 0.0,
            0.0, 1.0, 0.0,
            0.0, 0.0, 1.0,
            0.577350269190, 0.577350269190, 0.577350269190
        ]
        js_list = [0.1, 0.1, 0.1, 0.1]
        rw_config_params.GsMatrix_B = G_s_B
        rw_config_params.JsList = js_list
        rw_config_params.numRW = rw_num
        rw_param_in_msg = messaging.RWArrayConfigMsg().write(rw_config_params)

    # wheelAvailability message
    rw_availability_message = messaging.RWAvailabilityMsgPayload()
    if use_rw_availability != "NO":
        if use_rw_availability == "ON":
            rw_availability_message.wheelAvailability = [messaging.AVAILABLE, messaging.AVAILABLE,
                                                       messaging.AVAILABLE, messaging.AVAILABLE]
        elif use_rw_availability == "OFF":
            rw_availability_message.wheelAvailability = [messaging.UNAVAILABLE, messaging.UNAVAILABLE,
                                                       messaging.UNAVAILABLE, messaging.UNAVAILABLE]
        else:
            print("WARNING: unknown rw availability status")
        rw_avail_in_msg = messaging.RWAvailabilityMsg().write(rw_availability_message)
    else:
        # set default availability
        rw_availability_message.wheelAvailability = [messaging.AVAILABLE, messaging.AVAILABLE,
                                                   messaging.AVAILABLE, messaging.AVAILABLE]

    Lr_true = find_true_torques(module, guid_cmd_data, rw_speed_message, vehicle_config, js_list,
                                rw_num, G_s_B, rw_availability_message, ctrl_law)

    #   Setup logging on the test module output message so that we get all the writes to it
    data_log = module.cmdTorqueOutMsg.recorder()
    unit_test_sim.AddModelToTask(unit_task_name, data_log)

    # connect messages
    module.guidInMsg.subscribeTo(guid_in_msg)
    module.vehConfigInMsg.subscribeTo(vc_in_msg)
    if rw_num > 0:
        module.rwParamsInMsg.subscribeTo(rw_param_in_msg)
        module.rwSpeedsInMsg.subscribeTo(rw_speed_in_msg)
    if use_rw_availability != "NO":
        module.rwAvailInMsg.subscribeTo(rw_avail_in_msg)

    #   Need to call the self-init and cross-init methods
    unit_test_sim.InitializeSimulation()

    #   Step the simulation to 3*process rate so 4 total steps including zero
    unit_test_sim.ConfigureStopTime(macros.sec2nano(1.0))        # seconds to stop simulation
    unit_test_sim.ExecuteSimulation()

    module.reset(1)     # this module reset function needs a time input (in NanoSeconds)

    unit_test_sim.ConfigureStopTime(macros.sec2nano(2.0))        # seconds to stop simulation
    unit_test_sim.ExecuteSimulation()

    Lr = data_log.torqueRequestBody

    # compare the module results to the truth values
    accuracy = 1e-8
    np.testing.assert_allclose(Lr, Lr_true, atol=accuracy, rtol=0, verbose=True)


def find_true_torques(module, guid_cmd_data, rw_speed_message, vehicle_config_out, js_list, num_rw, G_s_B, rw_avail_msg, ctrl_law):
    Lr = []

    #Read in variables
    L = np.asarray(module.getKnownTorquePntB_B()).flatten()
    steps = [0, 0, .5, 0, .5]
    omega_BR_B = np.asarray(guid_cmd_data.omega_BR_B)
    omega_RN_B = np.asarray(guid_cmd_data.omega_RN_B)
    omega_BN_B = omega_BR_B + omega_RN_B #find body rate
    domega_RN_B = np.asarray(guid_cmd_data.domega_RN_B)
    sigma_BR = np.asarray(guid_cmd_data.sigma_BR)
    Isc = np.asarray(vehicle_config_out.ISCPntB_B)
    Isc = np.reshape(Isc, (3, 3))
    Ki = module.getKi()
    K = module.getK()
    P = module.getP()
    js_vec = js_list
    G_s_B_array = np.asarray(G_s_B)
    G_s_B_array = np.reshape(G_s_B_array[0:num_rw * 3], (num_rw, 3))
    sigma_int = np.asarray([0, 0, 0])

    #Compute toruqes
    for i in range(len(steps)):
        dt = steps[i]
        if dt == 0:
            sigma_int = np.asarray([0, 0, 0])

        #evaluate integral term
        if Ki > 0: #if integral feedback is on
            sigma_int = K * dt * sigma_BR + sigma_int
            for n in range(3):
                if abs(sigma_int[n]) > module.getIntegralLimit():
                    sigma_int[n] *= module.getIntegralLimit()/sigma_int[n] #check elementwise if integral term is greater than limit; preserve direction (+/-)

            z_vec = sigma_int + Isc.dot(omega_BR_B)
        else: #integral gain turned off/negative setting
            z_vec = np.asarray([0, 0, 0])

        #compute torque Lr
        Lr0 = K * sigma_BR  # +K*sigmaBR
        Lr1 = Lr0 + P * omega_BR_B  # +P*deltaOmega
        Lr2 = Lr1 + P * Ki * z_vec  # +P*Ki*z
        GsHs = np.array([0,0,0])

        if num_rw>0:
            for i in range(num_rw):
                if rw_avail_msg.wheelAvailability[i] == 0:  #Make RW availability check
                    GsHs = GsHs + np.dot(G_s_B_array[i, :], js_vec[i] * (np.dot(omega_BN_B, G_s_B_array[i, :]) + rw_speed_message.wheelSpeeds[i]))
                    #J_s*(dot(omegaBN_B,Gs_vec)+Omega_wheel)

        if ctrl_law == 0:
            Lr3 = Lr2 - np.cross((omega_RN_B+Ki*z_vec), (Isc.dot(omega_BN_B)+GsHs)) # -[v3Tilde(omega_r+Ki*z)]([I]omega + [Gs]h_s)
        else:
            Lr3 = Lr2 - np.cross(omega_BN_B, (Isc.dot(omega_BN_B)+GsHs)) # -[v3Tilde(omega)]([I]omega + [Gs]h_s)

        Lr4 = Lr3 + Isc.dot(-domega_RN_B + np.cross(omega_BN_B, omega_RN_B)) #+[I](-d(omega_r)/dt + omega x omega_r)
        Lr5 = Lr4 + L
        Lr5 = -Lr5
        Lr.append(np.ndarray.tolist(Lr5))
    return np.array(Lr)


if __name__ == "__main__":
    test_mrp_feedback(False,  # showplots
                      0.01,  # intGain
                      0,  # rwNum
                      0.0,  # integralLimit
                      1,  # ctrlLaw
                      "NO"  # useRwAvailability ("NO", "ON", "OFF")
                      )
