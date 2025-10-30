import sys

import numpy as np
import pytest
from datetime import datetime
from xmera.architecture import messaging
from xmera.fswAlgorithms import forceTorqueThrForceMapping
from xmera.utilities import SimulationBaseClass
from xmera.utilities import fswSetupThrusters
from xmera.utilities import macros

rcs_location_data_1 = [[-0.86360, -0.82550, 1.79070],
                       [-0.82550, -0.86360, 1.79070],
                       [0.82550, 0.86360, 1.79070],
                       [0.86360, 0.82550, 1.79070],
                       [-0.86360, -0.82550, -1.79070],
                       [-0.82550, -0.86360, -1.79070],
                       [0.82550, 0.86360, -1.79070],
                       [0.86360, 0.82550, -1.79070]]

rcs_location_data_2 = [[-1, -1, 1],
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

rcs_direction_data_1 = [[1.0, 0.0, 0.0],
                        [0.0, 1.0, 0.0],
                        [0.0, -1.0, 0.0],
                        [-1.0, 0.0, 0.0],
                        [1.0, 0.0, 0.0],
                        [0.0, 1.0, 0.0],
                        [0.0, -1.0, 0.0],
                        [-1.0, 0.0, 0.0]]

rcs_direction_data_2 = [[1.0, 0.0, 0.0],
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

# Use today's date as the seed for the random number generator. This ensures that several pytest threads receive the
# same randomized parameter inputs, while testing different randomized values over time. That number is rounded to the
# closest day based on the hour such that threads started close to midnight still result in the same seed number.
date_string = datetime.now().strftime("%Y%m%d")
date_number = float(date_string)
date_rounded = int(round(date_number + datetime.now().hour/24))
np.random.seed(date_rounded)

num_thr_rand = np.random.randint(1, messaging.MAX_EFF_CNT)
rcs_location_data_rand = np.round(np.random.randn(num_thr_rand, 3), 3).tolist()
randomized_directions = np.round(np.random.randn(num_thr_rand, 3), 3)
rcs_direction_data_rand = (randomized_directions / np.linalg.norm(randomized_directions, axis=1)[:, None]).tolist()
torque_rand = np.random.rand(3)
force_rand = np.random.rand(3)

r"""
Test 1: Ensures that the forceTorqueThrForce module can compute a valid solution for cases where there is a direction
        where no thrusters point - ensures matrix invertibility is handled.
Test 2: Ensures that the forceTorqueThrForce module can compute a valid solution for the case where there is zero
        requested torque in a connected input message, but a requested non-zero force.
Test 3: Ensures that the forceTorqueThrForce module can compute a valid solution for the case where there is no torque
        input message, but a requested non-zero force.
Test 4: Ensures that the forceTorqueThrForce module can compute a valid solution for the case where Thrusters point in
        each direction.
"""

@pytest.mark.parametrize("rcs_location, rcs_direction, requested_torque, requested_force, torque_in_msg_flag",
                         [(rcs_location_data_1, rcs_direction_data_1, [0.4, 0.2, 0.4], [0.9, 1.1, 0.], True),
                          (rcs_location_data_1, rcs_direction_data_1, [0.0, 0.0, 0.0], [0.9, 1.1, 0.], True),
                          (rcs_location_data_1, rcs_direction_data_1, [0.0, 0.0, 0.0], [0.9, 1.1, 0.], False),
                          (rcs_location_data_2, rcs_direction_data_2, [0.0, 0.0, 0.0], [0.9, 1.1, 1.], True),
                          (rcs_location_data_rand, rcs_direction_data_rand, torque_rand, force_rand, True)])

@pytest.mark.skipif(sys.platform == "win32", reason="known to not pass on windows platform")
def test_force_torque_thr_force_mapping(rcs_location, rcs_direction, requested_torque, requested_force,
                                        torque_in_msg_flag):
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

    CoM_B = np.array([0.1, 0.1, 0.1])

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

    truth = compute_thrust_mapping_truth(rcs_location, rcs_direction, requested_torque, requested_force, CoM_B)

    accuracy = 1e-12
    np.testing.assert_allclose(np.array([module.thrForceCmdOutMsg.read().thrForce[0:len(rcs_location)]]).flatten(), truth,
                               atol=accuracy, rtol=0, verbose=True)


def compute_thrust_mapping_truth(rcs_location, rcs_direction, requested_torque, requested_force, CoM_B):
    F = np.array([requested_torque, requested_force]).flatten()

    r_thr_B = np.array(rcs_location)
    r_M_B = np.array(CoM_B)
    r_thrM_B = r_thr_B - r_M_B
    gt_thr_B = np.array(rcs_direction)

    tau_B = np.cross(r_thrM_B, gt_thr_B, axis=1)

    DG = np.vstack([tau_B.T, gt_thr_B.T])

    DG_nonzero = []
    F_nonzero = []
    for i in range(6):
        if np.any(abs(DG[i]) > 1e-7):
            DG_nonzero.append(DG[i].tolist())
            F_nonzero.append(F[i].tolist())

    DG_nonzero = np.array(DG_nonzero)
    F_nonzero = np.array(F_nonzero)

    thr_force = np.linalg.pinv(DG_nonzero) @ F_nonzero
    thr_force -= np.min(thr_force)

    return thr_force


if __name__ == "__main__":
    test_force_torque_thr_force_mapping(rcs_location_data_1,
                                        rcs_direction_data_1,
                                        [0.4, 0.2, 0.4],
                                        [0.9, 1.1, 0.],
                                        True)
