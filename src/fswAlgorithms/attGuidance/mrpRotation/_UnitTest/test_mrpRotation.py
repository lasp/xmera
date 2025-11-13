import inspect
import os

import pytest

filename = inspect.getframeinfo(inspect.currentframe()).filename
path = os.path.dirname(os.path.abspath(filename))

import numpy as np

from xmera.utilities import SimulationBaseClass
from xmera.fswAlgorithms import mrpRotation
from xmera.utilities import macros as mc
from xmera.utilities import RigidBodyKinematics as rbk
from xmera.architecture import messaging


def compute_truth(sigma_RR0, omega_RR0_R, ref_state_in_data, dt, cmd_state_flag, test_reset):
    truth_sigma = []
    truth_omega_RN_N = []
    truth_domega_RN_N = []

    sigma_R0N = ref_state_in_data.sigma_RN
    R0N = rbk.MRP2C(sigma_R0N)
    omega_R0N_N = ref_state_in_data.omega_RN_N
    domega_R0N_N = ref_state_in_data.domega_RN_N

    # compute 0th time step
    s0 = np.array(sigma_RR0)
    s1=rbk.addMRP(np.array(sigma_R0N), np.array(sigma_RR0))
    RR0 = rbk.MRP2C(sigma_RR0)
    RN = np.dot(RR0, R0N)

    omega_RR0_N = np.dot(RN.T, omega_RR0_R)
    omega_RN_N = omega_RR0_N + omega_R0N_N

    domega_RR0_N = np.cross(omega_R0N_N, omega_RR0_N)
    domega_RN_N = domega_RR0_N + domega_R0N_N

    truth_sigma.append(s1.tolist())
    truth_omega_RN_N.append(omega_RN_N.tolist())
    truth_domega_RN_N.append(domega_RN_N.tolist())
    truth_sigma.append(s1.tolist())
    truth_omega_RN_N.append(omega_RN_N.tolist())
    truth_domega_RN_N.append(domega_RN_N.tolist())

    # compute 1st time step
    B =  rbk.BmatMRP(sigma_RR0)
    sigma_RR0 += dt * 0.25 * np.dot(B, omega_RR0_R)
    RR0 = rbk.MRP2C(sigma_RR0)
    RN = np.dot(RR0, R0N)
    sigma_RN = rbk.C2MRP(RN)
    truth_sigma.append(sigma_RN.tolist())

    omega_RR0_N = np.dot(RN.T, omega_RR0_R)
    omega_RN_N = omega_RR0_N + omega_R0N_N
    truth_omega_RN_N.append(omega_RN_N.tolist())

    domega_RR0_N = np.cross(omega_R0N_N, omega_RR0_N)
    domega_RN_N = domega_RR0_N + domega_R0N_N
    truth_domega_RN_N.append(domega_RN_N.tolist())

    # compute 2nd time step
    B =  rbk.BmatMRP(sigma_RR0)
    sigma_RR0 += dt * 0.25 * np.dot(B, omega_RR0_R)
    RR0 = rbk.MRP2C(sigma_RR0)
    RN = np.dot(RR0, R0N)
    sigma_RN = rbk.C2MRP(RN)
    truth_sigma.append(sigma_RN.tolist())

    omega_RR0_N = np.dot(RN.T, omega_RR0_R)
    omega_RN_N = omega_RR0_N + omega_R0N_N
    truth_omega_RN_N.append(omega_RN_N.tolist())

    domega_RR0_N = np.cross(omega_R0N_N, omega_RR0_N)
    domega_RN_N = domega_RR0_N + domega_R0N_N
    truth_domega_RN_N.append(domega_RN_N.tolist())

    # Testing Reset function
    if test_reset:
        if cmd_state_flag:
            sigma_RR0 = s0
        # compute 0th time step
        s1 = rbk.addMRP(np.array(sigma_R0N), np.array(sigma_RR0))
        RR0 = rbk.MRP2C(sigma_RR0)
        RN = np.dot(RR0, R0N)

        omega_RR0_N = np.dot(RN.T, omega_RR0_R)
        omega_RN_N = omega_RR0_N + omega_R0N_N

        domega_RR0_N = np.cross(omega_R0N_N, omega_RR0_N)
        domega_RN_N = domega_RR0_N + domega_R0N_N

        truth_sigma.append(s1.tolist())
        truth_omega_RN_N.append(omega_RN_N.tolist())
        truth_domega_RN_N.append(domega_RN_N.tolist())

        # compute 1st time step
        B = rbk.BmatMRP(sigma_RR0)
        sigma_RR0 += dt * 0.25 * np.dot(B, omega_RR0_R)
        RR0 = rbk.MRP2C(sigma_RR0)
        RN = np.dot(RR0, R0N)
        sigma_RN = rbk.C2MRP(RN)
        truth_sigma.append(sigma_RN.tolist())

        omega_RR0_N = np.dot(RN.T, omega_RR0_R)
        omega_RN_N = omega_RR0_N + omega_R0N_N
        truth_omega_RN_N.append(omega_RN_N.tolist())

        domega_RR0_N = np.cross(omega_R0N_N, omega_RR0_N)
        domega_RN_N = domega_RR0_N + domega_R0N_N
        truth_domega_RN_N.append(domega_RN_N.tolist())

    return truth_sigma, truth_omega_RN_N, truth_domega_RN_N


@pytest.mark.parametrize("cmd_state_flag", [False, True])
@pytest.mark.parametrize("test_reset", [False, True])
# provide a unique test method name, starting with test_
def test_mrp_rotation(show_plots, cmd_state_flag, test_reset):
    unit_task_name = "unitTask"               # arbitrary name (don't change)
    unit_process_name = "TestProcess"         # arbitrary name (don't change)
    unit_test_sim = SimulationBaseClass.SimBaseClass()

    # Test times
    update_time = 0.5     # update process rate update time
    total_test_sim_time = 1.5

    # Create test thread
    test_process_rate = mc.sec2nano(update_time)
    test_proc = unit_test_sim.CreateNewProcess(unit_process_name)
    test_proc.addTask(unit_test_sim.CreateNewTask(unit_task_name, test_process_rate))

    # Construct algorithm and associated C++ container
    module = mrpRotation.MrpRotation()
    module.modelTag = "mrpRotation"

    # Add test module to runtime call list
    unit_test_sim.AddModelToTask(unit_task_name, module)

    # Initialize the test module configuration data
    sigma_RR0 = np.array([0.3, .5, 0.0])
    module.sigma_RR0 = sigma_RR0
    omega_RR0_R = np.array([0.1, 0.0, 0.0]) * mc.D2R
    module.omega_RR0_R = omega_RR0_R

    if cmd_state_flag:
        desired_att = messaging.AttStateMsgPayload()
        sigma_RR0 = np.array([0.1, 0.0, -0.2])
        desired_att.state = sigma_RR0
        omega_RR0_R = np.array([0.1, 1.0, 0.5]) * mc.D2R
        desired_att.rate = omega_RR0_R
        des_in_msg = messaging.AttStateMsg().write(desired_att)
        module.desiredAttInMsg.subscribeTo(des_in_msg)

    # Reference Frame Message
    ref_state_in_data = messaging.AttRefMsgPayload()  # Create a structure for the input message
    sigma_R0N = np.array([0.1, 0.2, 0.3])
    ref_state_in_data.sigma_RN = sigma_R0N
    omega_R0N_N = np.array([0.1, 0.0, 0.0])
    ref_state_in_data.omega_RN_N = omega_R0N_N
    domega_R0N_N = np.array([0.0, 0.0, 0.0])
    ref_state_in_data.domega_RN_N = domega_R0N_N
    att_ref_msg = messaging.AttRefMsg().write(ref_state_in_data)
    module.attRefInMsg.subscribeTo(att_ref_msg)

    # Setup logging on the test module output message so that we get all the writes to it
    data_log = module.attRefOutMsg.recorder()
    unit_test_sim.AddModelToTask(unit_task_name, data_log)

    unit_test_sim.InitializeSimulation()
    unit_test_sim.ConfigureStopTime(mc.sec2nano(total_test_sim_time))        # seconds to stop simulation
    unit_test_sim.ExecuteSimulation()

    if test_reset:
        module.reset(1)
        unit_test_sim.ConfigureStopTime(mc.sec2nano(total_test_sim_time+1.0))        # seconds to stop simulation
        unit_test_sim.ExecuteSimulation()

    sigma_RN_true, omega_RN_true, dOmega_RN_true = compute_truth(sigma_RR0, omega_RR0_R, ref_state_in_data, update_time, cmd_state_flag, test_reset)

    accuracy = 1e-12

    np.testing.assert_allclose(data_log.sigma_RN, sigma_RN_true, atol=accuracy, rtol=0, verbose=True)
    np.testing.assert_allclose(data_log.omega_RN_N, omega_RN_true, atol=accuracy, rtol=0, verbose=True)
    np.testing.assert_allclose(data_log.domega_RN_N, dOmega_RN_true, atol=accuracy, rtol=0, verbose=True)


if __name__ == "__main__":
    test_mrp_rotation(False, False, True)
