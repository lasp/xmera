import pytest

from xmera.utilities import SimulationBaseClass
from xmera.fswAlgorithms import thrFiringRemainder
from xmera.utilities import macros
from xmera.utilities import fswSetupThrusters
from xmera.architecture import messaging

import numpy as np

@pytest.mark.parametrize("reset_check, dv_on", [
    (False,False),
    (True,False),
    (False,True),
    (True,True)
])
def test_thrFiringRemainder(show_plots, reset_check, dv_on):

    unit_task_name = "unitTask"
    unit_process_name = "TestProcess"

    unit_test_sim = SimulationBaseClass.SimBaseClass()

    fsw_rate = 0.5
    default_control_period = 3.0
    test_process_rate = macros.sec2nano(fsw_rate)  # update process rate update time
    test_proc = unit_test_sim.CreateNewProcess(unit_process_name)
    test_proc.addTask(unit_test_sim.CreateNewTask(unit_task_name, test_process_rate))

    # Construct algorithm and associated C++ container
    module = thrFiringRemainder.ThrFiringRemainder()
    module.modelTag = "thrFiringRemainder"

    # Add test module to runtime call list
    unit_test_sim.AddModelToTask(unit_task_name, module)

    # Initialize the test module configuration data
    module.thrMinFireTime = 0.2
    module.defaultControlPeriod = default_control_period
    if dv_on == 1:
        module.thrustPulsingRegime = thrFiringRemainder.ThrustPulsingRegime_OFF_PULSING
    else:
        module.thrustPulsingRegime = thrFiringRemainder.ThrustPulsingRegime_ON_PULSING

    # setup thruster cluster message
    fswSetupThrusters.clearSetup()
    rcs_location_data = [
        [-0.86360, -0.82550, 1.79070],
        [-0.82550, -0.86360, 1.79070],
        [0.82550, 0.86360, 1.79070],
        [0.86360, 0.82550, 1.79070],
        [-0.86360, -0.82550, -1.79070],
        [-0.82550, -0.86360, -1.79070],
        [0.82550, 0.86360, -1.79070],
        [0.86360, 0.82550, -1.79070]
        ]
    rcs_direction_data = [
        [1.0, 0.0, 0.0],
        [0.0, 1.0, 0.0],
        [0.0, -1.0, 0.0],
        [-1.0, 0.0, 0.0],
        [-1.0, 0.0, 0.0],
        [0.0, -1.0, 0.0],
        [0.0, 1.0, 0.0],
        [1.0, 0.0, 0.0]
        ]
    max_thrust = 0.5

    for i in range(len(rcs_location_data)):
        fswSetupThrusters.create(rcs_location_data[i], rcs_direction_data[i], max_thrust)
    thr_config_msg = fswSetupThrusters.writeConfigMessage()
    num_thrusters = fswSetupThrusters.getNumOfDevices()

    # setup thruster impulse request message
    thr_message_data = messaging.THRArrayCmdForceMsgPayload()
    if dv_on:
        thr_message_data.thrForce = [-0.5, 0.0, -0.1, -0.2, -0.3, -0.34, -0.39, -0.44]
    else:
        thr_message_data.thrForce = [0.5, 0.05, 0.1, 0.15, 0.19, 0.0, 0.2, 0.49]
    thr_force_msg = messaging.THRArrayCmdForceMsg().write(thr_message_data)

    # Setup logging on the test module output message so that we get all the writes to it
    data_log = module.onTimeOutMsg.recorder()
    unit_test_sim.AddModelToTask(unit_task_name, data_log)

    # connect messages
    module.thrConfInMsg.subscribeTo(thr_config_msg)
    module.thrForceInMsg.subscribeTo(thr_force_msg)

    # Need to call the self-init and cross-init methods
    unit_test_sim.InitializeSimulation()

    # Set the simulation time.
    # NOTE: the total simulation time may be longer than this value. The
    # simulation is stopped at the next logging event on or after the
    # simulation end time.
    final_time = 3.0
    unit_test_sim.ConfigureStopTime(macros.sec2nano(final_time))  # seconds to stop simulation
    unit_test_sim.ExecuteSimulation()

    if reset_check:
        # reset the module to test this functionality
        module.reset(macros.sec2nano(final_time))  # this module reset function needs a time input (in NanoSeconds)

        # run the module again for an additional 2.5 seconds
        unit_test_sim.ConfigureStopTime(macros.sec2nano(5.5))  # seconds to stop simulation
        unit_test_sim.ExecuteSimulation()

    module_output = data_log.OnTimeRequest[:, :num_thrusters]

    # compute true values
    thr_force = thr_message_data.thrForce
    pulse_remainder = np.zeros([num_thrusters])
    true_vector = np.empty([len(module_output), num_thrusters])
    idx_reset = final_time / fsw_rate + 1
    for idx in range(0, len(module_output)):
        on_times = np.empty([num_thrusters])
        # reset at corresponding idx if resetCheck is true,
        # or at idx 0 and 1 as output is the same for time 0 and first time step
        if (reset_check and idx == idx_reset) or idx <= 1:
            control_period = default_control_period
            pulse_remainder = np.zeros([num_thrusters])  # reset pulse remainder
        else:
            control_period = fsw_rate

        for thr_idx in range(0, num_thrusters):
            thrust = thr_force[thr_idx]
            if dv_on:
                thrust += max_thrust
            thrust = max(thrust, 0.0)  # Do not allow thrust requests less than zero
            on_time = thrust / max_thrust * control_period
            on_time += pulse_remainder[thr_idx] * module.thrMinFireTime
            pulse_remainder[thr_idx] = 0.0
            if on_time < module.thrMinFireTime:
                pulse_remainder[thr_idx] = on_time / module.thrMinFireTime
                on_time = 0.0
            elif on_time >= control_period:
                on_time = 1.1 * control_period
            on_times[thr_idx] = on_time
        true_vector[idx] = on_times

    np.testing.assert_allclose(true_vector, module_output, atol=1e-12, verbose=True)


#
# This statement below ensures that the unitTestScript can be run as a
# stand-along python script
#
if __name__ == "__main__":
    test_thrFiringRemainder(True, False, False)
