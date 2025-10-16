"""
Module Name:        rwMotorTorque
"""

import inspect
import os

import numpy as np
import pytest

filename = inspect.getframeinfo(inspect.currentframe()).filename
path = os.path.dirname(os.path.abspath(filename))

# Import all of the modules that we are going to be called in this simulation
from xmera.utilities import SimulationBaseClass
from xmera.fswAlgorithms import rwMotorTorque
from xmera.utilities import macros
from xmera.architecture import messaging

@pytest.mark.parametrize("num_control_axes", [0, 1, 2, 3])
@pytest.mark.parametrize("num_wheels", [2, 4, messaging.RW_EFF_CNT])
@pytest.mark.parametrize("num_input_cmd_torques", [1, 2])
@pytest.mark.parametrize("rw_avail_msg",["NO", "ON", "OFF", "MIXED"])

def test_rw_motor_torque(show_plots, num_control_axes, num_wheels, num_input_cmd_torques, rw_avail_msg):
    # @TODO With the current implementation of throwing an exception when zero control axes are specified, Python quits
    #  and causes all unit tests to fail. Until a different way of handling exceptions or errors is implemented, the
    #  test with 0 control axes is skipped.
    if num_control_axes == 0:
        pytest.skip("Zero control axes can currently not be tested.")

    # In case compile-time max RW number is less than parametrized number of wheels, skip test
    if num_wheels > messaging.RW_EFF_CNT:
        pytest.skip("Number of reaction wheels greater than compile time RW_EFF_CNT.")

    unit_task_name = "unitTask"
    unit_process_name = "TestProcess"
    unit_test_sim = SimulationBaseClass.SimBaseClass()

    # Create test thread
    test_process_rate = macros.sec2nano(0.5)     # update process rate update time
    test_proc = unit_test_sim.CreateNewProcess(unit_process_name)
    test_proc.addTask(unit_test_sim.CreateNewTask(unit_task_name, test_process_rate))

    module = rwMotorTorque.RwMotorTorque()
    module.modelTag = "rwMotorTorque"

    # Initialize module variables
    if num_control_axes == 3:
        control_axes_B = [[1, 0, 0], [0, 1, 0], [0, 0, 1]]
    elif num_control_axes == 2:
        control_axes_B = [[1, 0, 0], [0, 1, 0], [0, 0, 0]]
    elif num_control_axes == 1:
        control_axes_B = [[1, 0, 0], [0, 0, 0], [0, 0, 0]]
    else:
        control_axes_B = [[0, 0, 0], [0, 0, 0], [0, 0, 0]]

    module.controlAxes_B = control_axes_B

    # Add test module to runtime call list
    unit_test_sim.AddModelToTask(unit_task_name, module)

    # attControl message
    input_message_data = messaging.CmdTorqueBodyMsgPayload()
    requested_torque1 = [1.0, -0.5, 0.7]
    input_message_data.torqueRequestBody = requested_torque1
    cmd_torque_in_msg = messaging.CmdTorqueBodyMsg().write(input_message_data)

    requested_torque = np.array(requested_torque1)

    if num_input_cmd_torques == 2:
        input_message_data2 = messaging.CmdTorqueBodyMsgPayload()
        requested_torque2 = [0.5, 1.0, 3.0]
        input_message_data2.torqueRequestBody = requested_torque2
        cmd_torque_in2_msg = messaging.CmdTorqueBodyMsg().write(input_message_data2)
        requested_torque += np.array(requested_torque2)

    # wheelConfigData message
    rw_config_params = messaging.RWArrayConfigMsgPayload()
    RW_EFF_CNT = messaging.RW_EFF_CNT

    if num_wheels == RW_EFF_CNT:
        rw_config_params.GsMatrix_B = [
            0.4835867893995201, 0.7025829597277155, 0.5220354411517549,
            0.6274167231454653, 0.4634123147571517, 0.6257773422303058,
            0.4927675437195689, 0.3909468277672152, 0.7773935462269635,
            0.2791305379092009, 0.20278639222840245, 0.9385967301954065,
            0.1742148051521812, 0.9353106472878886, 0.3079662233682429,
            0.7408864742367625, 0.30733781515416325, 0.5971856492492805,
            0.49166240509756476, 0.11024265612126483, 0.863779275153674,
            0.08522980139648922, 0.5635691254043687, 0.8216603445736381,
            0.5169183283391889, 0.6482094982986043, 0.5591242153068406,
            0.5539478507672101, 0.4352935184619988, 0.7096910112262675,
            0.08177103922211226, 0.7185493168899821, 0.6906521384470449,
            0.5424303480563135, 0.8034905566669417, 0.24530031156636306,
            0.6791649825098244, 0.25103926707369056, 0.6897203874901293,
            0.6662787689368599, 0.6695372377111813, 0.32831766535181106,
            0.28428078464167594, 0.5440295499812461, 0.7894404880867942,
            0.8881073966834958, 0.007176386091829566, 0.4595799728433832,
            0.7043700914244455, 0.20398698108861654, 0.6798912308987893,
            0.5913513581668906, 0.7154722881784563, 0.3720255045596441,
            0.5353927164036736, 0.8292977052562882, 0.1600623480977027,
            0.5626385603464779, 0.5530980227747188, 0.6144269099038059,
            0.8047402627946283, 0.5179828986694456, 0.2899772855298006,
            0.6435726414836709, 0.49863310510036174, 0.5806714059015666,
            0.2533767502100278, 0.8066673674024603, 0.533936307831739,
            0.051675625147813466, 0.741898369799065, 0.6685180914942186,
            0.6705007071467579, 0.243658731626882, 0.700756180292173,
            0.6124322825812726, 0.6044312394389204, 0.5094993386086216,
            0.5025822950964116, 0.49662160344788164, 0.7076567103083798,
            0.4875326918964735, 0.8575174427431412, 0.16424283766253403,
            0.3659744927810267, 0.8415919620749859, 0.39722240622155974,
            0.6205921515961875, 0.5508152351685801, 0.5580931446303532,
            0.20125257120061574, 0.7022636474963218, 0.6828785924235018,
            0.4318909377763495, 0.6786025351852008, 0.5941117883924572,
            0.6839787443692367, 0.6598940110591041, 0.31098709204629277,
            0.35743175000357147, 0.8343049491885353, 0.4197353878920623,
            0.8124751056450826, 0.35669421673672336, 0.46114362020262967,
            0.04721328350343224, 0.8901899787392832, 0.45313652204714083]
    else:
        rw_config_params.GsMatrix_B = [
            1.0, 0.0, 0.0,
            0.0, 1.0, 0.0,
            0.0, 0.0, 1.0,
            0.5773502691896258, 0.5773502691896258, 0.5773502691896258
        ]

    rw_config_params.JsList = [0.1] * num_wheels
    rw_config_params.numRW = num_wheels
    rw_config_in_msg = messaging.RWArrayConfigMsg().write(rw_config_params)

    if rw_avail_msg != "NO":
        rw_availability_message = messaging.RWAvailabilityMsgPayload()

        avail = [messaging.UNAVAILABLE] * num_wheels
        for i in range(num_wheels):
            if rw_avail_msg == "ON":
                avail[i] = messaging.AVAILABLE
            elif rw_avail_msg == "OFF":
                avail[i] = messaging.UNAVAILABLE
            else:
                if i < int(num_wheels / 2):
                    avail[i] = messaging.AVAILABLE

        rw_availability_message.wheelAvailability = avail

        rw_avail_in_msg = messaging.RWAvailabilityMsg().write(rw_availability_message)
        module.rwAvailInMsg.subscribeTo(rw_avail_in_msg)
    else:
        avail = [rwMotorTorque.AVAILABLE] * num_wheels  # this is used purely for the python level solution

    # Setup logging on the test module output message so that we get all the writes to it
    data_log = module.rwMotorTorqueOutMsg.recorder()
    unit_test_sim.AddModelToTask(unit_task_name, data_log)

    # connect messages
    module.vehControlInMsg.subscribeTo(cmd_torque_in_msg)
    if num_input_cmd_torques == 2:
        module.vehControlIn2Msg.subscribeTo(cmd_torque_in2_msg)
    module.rwParamsInMsg.subscribeTo(rw_config_in_msg)

    unit_test_sim.InitializeSimulation()
    module.reset(0)

    # Set the simulation time.
    unit_test_sim.ConfigureStopTime(macros.sec2nano(0.5))

    # Begin the simulation time run set above
    unit_test_sim.ExecuteSimulation()

    # This pulls the actual data log from the simulation run.
    motor_torque = data_log.motorTorque

    # set the output truth states
    u_s = compute_true_torque(np.array(control_axes_B),
                              np.array(rw_config_params.GsMatrix_B).reshape((3, RW_EFF_CNT), order='F'),
                              requested_torque,
                              avail)

    true_motor_torque = [u_s] * 2

    # compare the module results to the truth values
    accuracy = 1e-8
    np.testing.assert_allclose(motor_torque, true_motor_torque, rtol=0, atol=accuracy, verbose=True)

    G_s_B =np.array( rw_config_params.GsMatrix_B).reshape((3, RW_EFF_CNT), order='F')
    F = np.transpose(motor_torque[0])
    received_torque = -(G_s_B @ F).flatten()

    if num_wheels >= num_control_axes > 0:
        if (len(avail) - np.sum(avail)) > num_control_axes:
            np.testing.assert_allclose(received_torque[:num_control_axes], requested_torque[:num_control_axes],
                                       rtol=0,
                                       atol=accuracy,
                                       verbose=True)


def compute_true_torque(C, Gs_B, Lr, avail_msg):

    num_control_axes = (np.linalg.norm(C, axis=1) > 0.0).sum()
    num_wheels = len(avail_msg)
    non_avail_wheels = 0

    # Remove wheels that are deemed unavailable
    for i in range(len(Gs_B[0])): #
        if num_wheels > i:
            if avail_msg[i] != messaging.AVAILABLE:
                Gs_B[:,i] = [0.0, 0.0, 0.0]
                non_avail_wheels += 1
        else:
            Gs_B[:,i] = [0.0, 0.0, 0.0]

    # If fewer wheels than number of control axes, output no torque
    if (num_wheels-non_avail_wheels) < num_control_axes:
        return [0.0]*len(Gs_B[0])

    Lr_C = np.dot(C,Lr) # Project torque onto control axes
    CGs = np.dot(C, Gs_B) # Map the control axes onto the wheels

    # Build minimum norm framework
    M = np.dot(CGs, CGs.T)
    M_rep = np.identity(3) # Need to keep the matrix non-singular for inversion
    for i in range(0,num_control_axes):
        for j in range(0,num_control_axes):
            M_rep[i][j] = M[i][j]
    M_inv = np.linalg.inv(M_rep)

    # Remove projection to any non-defined control axes
    for i in range(num_control_axes,3):
        M_inv[i][i] = 0.0

    # Determine the solution
    L = np.dot(M_inv, Lr_C)

    # Map the solution to the wheels
    u_s = np.dot(CGs.T, L)

    return -u_s


if __name__ == "__main__":
    test_rw_motor_torque(False,
                         3,  # numControlAxes
                         36,  # numWheels
                         2,  # numInputCmdTorques
                         "NO"  # RWAvailMsg ("NO", "ON", "OFF")
                         )
