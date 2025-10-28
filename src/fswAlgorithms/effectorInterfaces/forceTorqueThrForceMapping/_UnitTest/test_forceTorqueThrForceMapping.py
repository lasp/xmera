import sys

import numpy as np
import pytest
from xmera.architecture import messaging
from xmera.fswAlgorithms import forceTorqueThrForceMapping
from xmera.utilities import SimulationBaseClass
from xmera.utilities import fswSetupThrusters
from xmera.utilities import macros

@pytest.mark.skipif(sys.platform == "win32", reason="known to not pass on windows platform")
def test_force_torque_thr_force_mapping1():
    r"""
    **Test Description**

    This pytest ensures that the forceTorqueThrForce module can compute a valid solution for cases where:
    1. There is a direction where no thrusters point - ensures matrix invertibility is handled

    """

    # Test 1 - No thrusters pointing in one direction, CoM offset
    rcs_location_data = [[-0.86360, -0.82550, 1.79070],
                            [-0.82550, -0.86360, 1.79070],
                            [0.82550, 0.86360, 1.79070],
                            [0.86360, 0.82550, 1.79070],
                            [-0.86360, -0.82550, -1.79070],
                            [-0.82550, -0.86360, -1.79070],
                            [0.82550, 0.86360, -1.79070],
                            [0.86360, 0.82550, -1.79070]]

    rcs_direction_data = [[1.0, 0.0, 0.0],
                             [0.0, 1.0, 0.0],
                             [0.0, -1.0, 0.0],
                             [-1.0, 0.0, 0.0],
                             [1.0, 0.0, 0.0],
                             [0.0, 1.0, 0.0],
                             [0.0, -1.0, 0.0],
                             [-1.0, 0.0, 0.0]]

    requested_torque = [0.4, 0.2, 0.4]

    requested_force = [0.9, 1.1, 0.]

    CoM_B = [0.1, 0.1, 0.1]

    truth = np.array([[0.7082, 0.5500, 0.0810, 0.1772, 0.6272, 0.6310, 0., 0.2582]])

    force_torque_thr_force_mapping_test_function(rcs_location_data, rcs_direction_data, requested_torque,
                                                 requested_force, CoM_B, truth, True)


@pytest.mark.skipif(sys.platform == "win32", reason="known to not pass on windows platform")
def test_force_torque_thr_force_mapping2():
    r"""
    **Test Description**

    This pytest ensures that the forceTorqueThrForce module can compute a valid solution for the case
    where there is zero requested torque in a connected input message, but a requested non-zero force

    """

    # Test 1 - No thrusters pointing in one direction, CoM offset
    rcs_location_data = [[-0.86360, -0.82550, 1.79070],
                       [-0.82550, -0.86360, 1.79070],
                       [0.82550, 0.86360, 1.79070],
                       [0.86360, 0.82550, 1.79070],
                       [-0.86360, -0.82550, -1.79070],
                       [-0.82550, -0.86360, -1.79070],
                       [0.82550, 0.86360, -1.79070],
                       [0.86360, 0.82550, -1.79070]]

    rcs_direction_data = [[1.0, 0.0, 0.0],
                        [0.0, 1.0, 0.0],
                        [0.0, -1.0, 0.0],
                        [-1.0, 0.0, 0.0],
                        [1.0, 0.0, 0.0],
                        [0.0, 1.0, 0.0],
                        [0.0, -1.0, 0.0],
                        [-1.0, 0.0, 0.0]]

    requested_force = [0.9, 1.1, 0.]

    CoM_B = [0.1, 0.1, 0.1]

    requested_torque = [0.0, 0.0, 0.0]

    truth = np.array([[0.5340, 0.5807, 0., 0.0588, 0.5088, 0.5500, 0.0307, 0.0840]])

    force_torque_thr_force_mapping_test_function(rcs_location_data, rcs_direction_data, requested_torque,
                                                 requested_force, CoM_B, truth, True)


@pytest.mark.skipif(sys.platform == "win32", reason="known to not pass on windows platform")
def test_force_torque_thr_force_mapping3():
    r"""
    **Test Description**

    This pytest ensures that the forceTorqueThrForce module can compute a valid solution for the case
    where there is no torque input message, but a requested non-zero force

    """

    # Test 1 - No thrusters pointing in one direction, CoM offset
    rcs_location_data = [[-0.86360, -0.82550, 1.79070],
                       [-0.82550, -0.86360, 1.79070],
                       [0.82550, 0.86360, 1.79070],
                       [0.86360, 0.82550, 1.79070],
                       [-0.86360, -0.82550, -1.79070],
                       [-0.82550, -0.86360, -1.79070],
                       [0.82550, 0.86360, -1.79070],
                       [0.86360, 0.82550, -1.79070]]

    rcs_direction_data = [[1.0, 0.0, 0.0],
                        [0.0, 1.0, 0.0],
                        [0.0, -1.0, 0.0],
                        [-1.0, 0.0, 0.0],
                        [1.0, 0.0, 0.0],
                        [0.0, 1.0, 0.0],
                        [0.0, -1.0, 0.0],
                        [-1.0, 0.0, 0.0]]

    requested_force = [0.9, 1.1, 0.]

    CoM_B = [0.1, 0.1, 0.1]

    requested_torque = [0.0, 0.0, 0.0]

    truth = np.array([[0.5340, 0.5807, 0., 0.0588, 0.5088, 0.5500, 0.0307, 0.0840]])

    force_torque_thr_force_mapping_test_function(rcs_location_data, rcs_direction_data, requested_torque,
                                                 requested_force, CoM_B, truth, False)


@pytest.mark.skipif(sys.platform == "win32", reason="known to not pass on windows platform")
def test_force_torque_thr_force_mapping4():
    r"""
    **Test Description**

    This pytest ensures that the forceTorqueThrForce module can compute a valid solution for the case where
    Thrusters point in each direction

    """

    rcs_location_data = [[-1, -1, 1],
                        [-1, -1, 1],
                        [-1, -1, 1],
                        [1, 1, 1],
                        [1, 1, 1],
                        [1, 1, 1],
                        [1, 1, -1],
                        [1, 1, -1],
                        [1, 1, -1],
                        [-1, -1, -1],
                        [-1, -1, -1],
                        [-1, -1, -1]]

    rcs_direction_data = [[1.0, 0.0, 0.0],
                        [0.0, 1.0, 0.0],
                        [0.0, 0.0, -1.0],
                        [0.0, 0.0, -1.0],
                        [0.0, -1.0, 0.0],
                        [-1.0, 0.0, 0.0],
                        [0.0, -1.0, 0.0],
                        [-1.0, 0.0, 0.0],
                        [0.0, 0.0, 1.0],
                        [1.0, 0.0, 0.0],
                        [0.0, 1.0, 0.0],
                        [0.0, 0.0, 1.0]]

    CoM_B = [0.1, 0.1, 0.1]
    requested_torque = [0.0, 0.0, 0.0]
    requested_force = [0.9, 1.1, 1.]

    truth = np.array([[0.5050, 0.5550, 0.0300, 0.0300, 0., 0.0600, 0.0050, 0.0550, 0.5300, 0.5100, 0.5500, 0.5300]])

    force_torque_thr_force_mapping_test_function(rcs_location_data, rcs_direction_data, requested_torque,
                                                 requested_force, CoM_B, truth, True)


def force_torque_thr_force_mapping_test_function(rcs_location, rcs_direction, requested_torque, requested_force, CoM_B,
                                                 truth, torque_in_msg_flag):
    unit_task_name = "unitTask"
    unit_process_name = "TestProcess"

    unit_test_sim = SimulationBaseClass.SimBaseClass()
    test_process_rate = macros.sec2nano(0.5)
    test_proc = unit_test_sim.CreateNewProcess(unit_process_name)
    test_proc.addTask(unit_test_sim.CreateNewTask(unit_task_name, test_process_rate))

    # setup module to be tested
    module = forceTorqueThrForceMapping.ForceTorqueThrForceMapping()
    module.modelTag = "forceTorqueThrForceMappingTag"
    unit_test_sim.AddModelToTask(unit_task_name, module)

    # Configure blank module input messages
    cmd_torque_in_msg_data = messaging.CmdTorqueBodyMsgPayload()
    cmd_torque_in_msg_data.torqueRequestBody = requested_torque
    cmd_torque_in_msg = messaging.CmdTorqueBodyMsg().write(cmd_torque_in_msg_data)

    cmd_force_in_msg_data = messaging.CmdForceBodyMsgPayload()
    cmd_force_in_msg_data.forceRequestBody = requested_force
    cmd_force_in_msg = messaging.CmdForceBodyMsg().write(cmd_force_in_msg_data)

    num_thrusters = len(rcs_location)
    max_thrust = 3.0  # N
    MAX_EFF_CNT = messaging.MAX_EFF_CNT
    rcs_location_data = np.zeros((MAX_EFF_CNT, 3))
    rcs_direction_data = np.zeros((MAX_EFF_CNT, 3))

    rcs_location_data[0:len(rcs_location)] = rcs_location

    rcs_direction_data[0:len(rcs_location)] = rcs_direction

    fswSetupThrusters.clearSetup()
    for i in range(num_thrusters):
        fswSetupThrusters.create(rcs_location_data[i], rcs_direction_data[i], max_thrust)
    thr_config_in_msg = fswSetupThrusters.writeConfigMessage()

    veh_config_in_msg_data = messaging.VehicleConfigMsgPayload()
    veh_config_in_msg_data.CoM_B = CoM_B
    veh_config_in_msg = messaging.VehicleConfigMsg().write(veh_config_in_msg_data)

    # subscribe input messages to module
    if torque_in_msg_flag:
        module.cmdTorqueInMsg.subscribeTo(cmd_torque_in_msg)
    module.cmdForceInMsg.subscribeTo(cmd_force_in_msg)
    module.thrConfigInMsg.subscribeTo(thr_config_in_msg)
    module.vehConfigInMsg.subscribeTo(veh_config_in_msg)

    unit_test_sim.InitializeSimulation()
    unit_test_sim.ConfigureStopTime(macros.sec2nano(0.5))
    unit_test_sim.ExecuteSimulation()

    accuracy = 1e-12
    np.testing.assert_allclose(np.array([module.thrForceCmdOutMsg.read().thrForce[0:len(rcs_location)]]), truth,
                               atol=accuracy, rtol=0, verbose=True)


if __name__ == "__main__":
    test_force_torque_thr_force_mapping1()
