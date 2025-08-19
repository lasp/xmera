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

import matplotlib.pyplot as plt
import numpy as np
import pytest
from Basilisk.architecture import messaging
from Basilisk.simulation import stepperMotor
from Basilisk.utilities import SimulationBaseClass
from Basilisk.utilities import macros


@pytest.mark.parametrize("motor_theta_init", [0.0 * macros.D2R, 10.0 * macros.D2R, -5.0 * macros.D2R])
@pytest.mark.parametrize("steps_commanded_1", [0, 10, -15])
@pytest.mark.parametrize("steps_commanded_2", [0, 10, -15])
@pytest.mark.parametrize("step_angle", [0.1 * macros.D2R, 0.5 * macros.D2R, 1.0 * macros.D2R])
@pytest.mark.parametrize("step_time", [0.3, 0.5, 1.0])
def test_stepper_motor_nominal(show_plots,
                               motor_theta_init,
                               steps_commanded_1,
                               steps_commanded_2,
                               step_angle,
                               step_time):
    r"""
    **Verification Test Description**

    This nominal unit test ensures that the stepper motor simulation module correctly actuates the stepper motor from
    an initial angle to a final reference angle, given an input integer number of commanded steps. The module outputs
    the motor scalar states (angle, angle rate, and acceleration) and the motor step count as a function of time.
    The motor actuation is simulated using a bang-bang acceleration profile. The motor acceleration is calculated in
    the module using the given motor step angle and step time constants. The capability for the motor to take both
    positive and negative steps is checked in this test by commanding both positive and negative steps to the module.

    **Test Parameters**

    Args:
        motor_theta_init (float): [rad] Initial stepper motor angle
        steps_commanded_1 (int): [steps] Number of steps commanded to the stepper motor (first command)
        steps_commanded_2 (int): [steps] Number of steps commanded to the stepper motor (second command)
        step_angle (float): [rad] Angle the stepper motor moves through for a single step (constant)
        step_time (float): [sec] Time required for a single motor step (constant)

    **Description of Variables Being Tested**

    This unit test checks that the final motor angle from the simulation matches the reference motor angle computed
    in this script. The test also checks that the final motor step count matches the numer of steps commanded to the
    module. The motor angle rate is also checked to be zero at the end of the simulation.

    """

    task_name = "unitTask"
    process_name = "TestProcess"
    test_sim = SimulationBaseClass.SimBaseClass()
    test_process_rate_sec = 0.01
    test_process_rate = macros.sec2nano(test_process_rate_sec)
    test_process = test_sim.CreateNewProcess(process_name)
    test_process.addTask(test_sim.CreateNewTask(task_name, test_process_rate))

    # Create the stepperMotor module
    stepper_motor = stepperMotor.StepperMotor()
    stepper_motor.modelTag = "StepperMotor"
    stepper_motor.setThetaInit(motor_theta_init)
    stepper_motor.setStepAngle(step_angle)
    stepper_motor.setStepTime(step_time)
    test_sim.AddModelToTask(task_name, stepper_motor)

    # Create the first stepperMotor input message
    motor_step_command_msg_data = messaging.MotorStepCommandMsgPayload()
    motor_step_command_msg_data.stepsCommanded = steps_commanded_1
    motor_step_command_msg = messaging.MotorStepCommandMsg().write(motor_step_command_msg_data)
    stepper_motor.motorStepCommandInMsg.subscribeTo(motor_step_command_msg)

    # Set up data logging
    stepper_motor_data_log = stepper_motor.stepperMotorOutMsg.recorder()
    test_sim.AddModelToTask(task_name, stepper_motor_data_log)

    # Run the simulation
    test_sim.InitializeSimulation()
    sim_time_1 = step_time * abs(steps_commanded_1)  # [s]
    sim_time_extra = 5.0  # [s]
    test_sim.ConfigureStopTime(macros.sec2nano(sim_time_1 + sim_time_extra))
    test_sim.ExecuteSimulation()

    # Create the second stepperMotor input message
    motor_step_command_msg_data = messaging.MotorStepCommandMsgPayload()
    motor_step_command_msg_data.stepsCommanded = steps_commanded_2
    motor_step_command_msg = messaging.MotorStepCommandMsg().write(motor_step_command_msg_data,
                                                                   test_sim.TotalSim.getCurrentNanos())
    stepper_motor.motorStepCommandInMsg.subscribeTo(motor_step_command_msg)

    # Run the simulation
    sim_time_2 = step_time * abs(steps_commanded_2) + 2 * test_process_rate_sec  # [s]
    test_sim.ConfigureStopTime(macros.sec2nano(sim_time_1 + sim_time_extra + sim_time_2))
    test_sim.ExecuteSimulation()

    # Extract the logged data for plotting and data comparison
    timespan = macros.NANO2SEC * stepper_motor_data_log.times()  # [s]
    theta = macros.R2D * stepper_motor_data_log.theta  # [deg]
    theta_dot = macros.R2D * stepper_motor_data_log.thetaDot  # [deg/s]
    theta_ddot = macros.R2D * stepper_motor_data_log.thetaDDot  # [deg/s^2]
    motor_step_count = stepper_motor_data_log.stepCount
    motor_steps_commanded = stepper_motor_data_log.stepsCommanded

    if show_plots:
        plot_results(timespan,
                     theta,
                     theta_dot,
                     theta_ddot,
                     motor_step_count,
                     motor_steps_commanded)
        plt.show()
    plt.close("all")

    accuracy = 1e-12

    # Check the motor states converge to the reference values for the first actuation
    motor_theta_final_index_1 = int(round(sim_time_1 / test_process_rate_sec))
    motor_theta_1_ref_true = motor_theta_init + (steps_commanded_1 * step_angle)
    np.testing.assert_allclose(theta[motor_theta_final_index_1],
                               macros.R2D * motor_theta_1_ref_true,
                               atol=accuracy,
                               verbose=True)
    np.testing.assert_allclose(theta_dot[motor_theta_final_index_1],
                               0.0,
                               atol=accuracy,
                               verbose=True)
    np.testing.assert_allclose(theta_ddot[motor_theta_final_index_1 + 1],
                               0.0,
                               atol=accuracy,
                               verbose=True)
    np.testing.assert_allclose(motor_step_count[motor_theta_final_index_1],
                               steps_commanded_1,
                               atol=accuracy,
                               verbose=True)

    # Check the motor states converge to the reference values for the second actuation
    motor_theta_final_index_2 = -1
    motor_theta_2_ref_true = motor_theta_1_ref_true + (steps_commanded_2 * step_angle)
    np.testing.assert_allclose(theta[motor_theta_final_index_2],
                               macros.R2D * motor_theta_2_ref_true,
                               atol=accuracy,
                               verbose=True)
    np.testing.assert_allclose(theta_dot[motor_theta_final_index_2],
                               0.0,
                               atol=accuracy,
                               verbose=True)
    np.testing.assert_allclose(theta_ddot[motor_theta_final_index_2],
                               0.0,
                               atol=accuracy,
                               verbose=True)
    np.testing.assert_allclose(motor_step_count[motor_theta_final_index_2],
                               steps_commanded_2,
                               atol=accuracy,
                               verbose=True)

def plot_results(timespan,
                 theta,
                 theta_dot,
                 theta_ddot,
                 motor_step_count,
                 motor_steps_commanded):

    # Plot motor angle
    plt.figure()
    plt.clf()
    plt.plot(timespan, theta, label=r"$\theta$")
    plt.title(r'Stepper Motor Angle $\theta$', fontsize=14)
    plt.ylabel('(deg)', fontsize=14)
    plt.xlabel('Time (s)', fontsize=14)
    plt.legend(loc='upper right', prop={'size': 12})
    plt.grid(True)

    # Plot motor theta_dot
    plt.figure()
    plt.clf()
    plt.plot(timespan, theta_dot, label=r"$\dot{\theta}$")
    plt.title(r'Stepper Motor Angle Rate $\dot{\theta}$', fontsize=14)
    plt.ylabel('(deg/s)', fontsize=14)
    plt.xlabel('Time (s)', fontsize=14)
    plt.legend(loc='upper right', prop={'size': 12})
    plt.grid(True)

    # Plot motor theta_ddot
    plt.figure()
    plt.clf()
    plt.plot(timespan, theta_ddot, label=r"$\ddot{\theta}$")
    plt.title(r'Stepper Motor Angular Acceleration $\ddot{\theta}$ ', fontsize=14)
    plt.ylabel('(deg/s$^2$)', fontsize=14)
    plt.xlabel('Time (s)', fontsize=14)
    plt.legend(loc='upper right', prop={'size': 12})
    plt.grid(True)

    # Plot steps commanded and motor steps taken
    plt.figure()
    plt.clf()
    plt.plot(timespan, motor_step_count, label='Step Count')
    plt.plot(timespan, motor_steps_commanded, '--', label='Commanded')
    plt.title(r'Motor Step History', fontsize=14)
    plt.ylabel('Steps', fontsize=14)
    plt.xlabel('Time (s)', fontsize=14)
    plt.legend(loc='upper right', prop={'size': 12})
    plt.grid(True)


if __name__ == "__main__":
    test_stepper_motor_nominal(
        True,
        0.0,  # [rad] motor_theta_init
        15,  # steps_commanded_1
        -15,  # steps_commanded_2
        1.0 * macros.D2R,  # [rad] step_angle
        1.0,  # [s] step_time
    )
