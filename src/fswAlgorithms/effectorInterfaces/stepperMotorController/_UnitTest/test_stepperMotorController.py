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

import inspect
import os

import numpy as np
import pytest
from Basilisk.architecture import messaging
from Basilisk.fswAlgorithms import stepperMotorController
from Basilisk.utilities import SimulationBaseClass
from Basilisk.utilities import macros

filename = inspect.getframeinfo(inspect.currentframe()).filename
path = os.path.dirname(os.path.abspath(filename))
bskName = 'Basilisk'
splitPath = path.split(bskName)

@pytest.mark.parametrize("motor_step_angle", [0.008 * macros.D2R, 0.01 * macros.D2R, 0.5 * macros.D2R])
@pytest.mark.parametrize("motor_step_time", [0.008, 0.1, 0.5])
@pytest.mark.parametrize("motor_theta_init", [-5.0 * macros.D2R, 0.0, 60.0 * macros.D2R])
@pytest.mark.parametrize("motor_theta_ref", [0.0, 10.6 * macros.D2R, 60.0051 * macros.D2R])
def test_stepper_motor_controller_nominal(show_plots, motor_step_angle, motor_step_time, motor_theta_init,
                                          motor_theta_ref):
    r"""
    **Validation Test Description**

    This nominal unit test ensures that the stepper motor controller module correctly determines the number of steps
    required to actuate from an initial angle to a final reference angle. The initial and reference motor angles are
    varied so that both positive and negative steps are required in this test. It must be noted that the motor angles
    are discretized by a constant ``motorStepAngle``; therefore the motor cannot simply actuate to any desired angle.
    The reference motor angles are chosen in this test so that several cases require the reference to be adjusted
    to the nearest multiple of the motor step angle. In other words, this test checks cases where the exact number of
    motor steps required to reach the reference exactly is not an integer. For these cases, the final result for the
    number of commanded motor steps is rounded to the nearest integer step and the corresponding motor reference
    angle is updated to the reachable value.

    **Test Parameters**

    Args:
        motor_step_angle (float): [rad] Step angle the motor rotates through for a single step (constant)
        motor_step_time (float): [sec] Time required for the motor to actuate through a single step (constant)
        motor_theta_init (float): [rad] Initial stepper motor angle
        motor_theta_ref (float): [rad] Reference stepper motor angle

    **Description of Variables Being Tested**

    The module-computed number of required stepper motor steps is checked to match the number of motor steps
    computed in this script.

    """

    unit_task_name = "unitTask"
    unit_process_name = "TestProcess"
    unit_test_sim = SimulationBaseClass.SimBaseClass()
    test_process_rate = macros.sec2nano(motor_step_time)
    test_proc = unit_test_sim.CreateNewProcess(unit_process_name)
    test_proc.addTask(unit_test_sim.CreateNewTask(unit_task_name, test_process_rate))

    # Create the stepperMotorController module
    motor_controller = stepperMotorController.StepperMotorController()
    motor_controller.modelTag = "stepperMotorController"
    motor_controller.setStepAngle(motor_step_angle)  # [rad]
    motor_controller.setStepTime(motor_step_time)  # [s]
    motor_controller.setThetaInit(motor_theta_init)  # [rad]
    unit_test_sim.AddModelToTask(unit_task_name, motor_controller)

    # Create the stepperMotorController input message
    hinged_rigid_body_message_data = messaging.HingedRigidBodyMsgPayload()
    hinged_rigid_body_message_data.theta = motor_theta_ref  # [rad]
    hinged_rigid_body_message = messaging.HingedRigidBodyMsg().write(hinged_rigid_body_message_data)
    motor_controller.motorRefAngleInMsg.subscribeTo(hinged_rigid_body_message)

    # Set up data logging
    motor_step_command_log = motor_controller.motorStepCommandOutMsg.recorder(test_process_rate)
    unit_test_sim.AddModelToTask(unit_task_name, motor_step_command_log)

    # Calculate required number of motor steps to achieve the reference angle
    if motor_theta_init > 0:
        steps_commanded_truth = (motor_theta_ref - (np.ceil(motor_theta_init / motor_step_angle) * motor_step_angle)) / motor_step_angle
    else:
        steps_commanded_truth = (motor_theta_ref - (np.floor(motor_theta_init / motor_step_angle) * motor_step_angle)) / motor_step_angle

    # If the reference motor angle is not a multiple of the motor step angle, the number of steps calculated is not an
    # integer and it must be rounded to the nearest integer step
    lower_step_fraction = steps_commanded_truth - np.floor(steps_commanded_truth)
    upper_step_fraction = np.ceil(steps_commanded_truth) - steps_commanded_truth
    if upper_step_fraction > lower_step_fraction:
        steps_commanded_truth = np.floor(steps_commanded_truth)
    else:
        steps_commanded_truth = np.ceil(steps_commanded_truth)

    # Compute the time required for the motor to actuate to the reference angle
    actuate_time = motor_step_time * np.abs(steps_commanded_truth)  # [s]

    # Run the simulation
    unit_test_sim.InitializeSimulation()
    unit_test_sim.ConfigureStopTime(macros.sec2nano(actuate_time))
    unit_test_sim.ExecuteSimulation()

    # Pull the logged motor step data
    steps_commanded_sim = motor_step_command_log.stepsCommanded

    # Check that the correct number of steps was calculated
    accuracy = 1e-12
    np.testing.assert_allclose(steps_commanded_sim[0],
                               steps_commanded_truth,
                               atol=accuracy,
                               verbose=True)


@pytest.mark.parametrize("motor_theta_ref", [-10.0, 275.0])
def test_stepper_motor_controller_invalid(show_plots, motor_theta_ref):
    r"""
    **Validation Test Description**

    This unit test ensures that the stepper motor controller module correctly outputs zero steps commanded when the
    reference motor angle is outside the motor actuation bounds.

    **Test Parameters**

    Args:
        motor_theta_ref (float): [rad] Reference stepper motor angle

    **Description of Variables Being Tested**

    The module-computed number of required stepper motor steps is checked to be zero in this test.

    """

    unit_task_name = "unitTask"
    unit_process_name = "TestProcess"
    unit_test_sim = SimulationBaseClass.SimBaseClass()
    test_process_rate = macros.sec2nano(1.0)
    test_proc = unit_test_sim.CreateNewProcess(unit_process_name)
    test_proc.addTask(unit_test_sim.CreateNewTask(unit_task_name, test_process_rate))

    # Create the stepperMotorController module
    motor_controller = stepperMotorController.StepperMotorController()
    motor_controller.modelTag = "stepperMotorController"
    motor_controller.setThetaMin(180.0 * macros.D2R)  # [rad]
    motor_controller.setThetaMin(0.0)  # [rad]
    unit_test_sim.AddModelToTask(unit_task_name, motor_controller)

    # Create the stepperMotorController input message
    hinged_rigid_body_message_data = messaging.HingedRigidBodyMsgPayload()
    hinged_rigid_body_message_data.theta = motor_theta_ref  # [rad]
    hinged_rigid_body_message = messaging.HingedRigidBodyMsg().write(hinged_rigid_body_message_data)
    motor_controller.motorRefAngleInMsg.subscribeTo(hinged_rigid_body_message)

    # Set up data logging
    motor_step_command_log = motor_controller.motorStepCommandOutMsg.recorder(test_process_rate)
    unit_test_sim.AddModelToTask(unit_task_name, motor_step_command_log)

    # Run the simulation
    unit_test_sim.InitializeSimulation()
    unit_test_sim.ConfigureStopTime(macros.sec2nano(3.0))
    unit_test_sim.ExecuteSimulation()

    # Pull the logged motor step data
    steps_commanded_sim = motor_step_command_log.stepsCommanded

    # Check that the correct number of steps was calculated
    accuracy = 1e-12
    np.testing.assert_allclose(steps_commanded_sim[0],
                               0,
                               atol=accuracy,
                               verbose=True)


@pytest.mark.parametrize("motor_theta_ref1", [-10.0 * macros.D2R, 10.0 * macros.D2R])
@pytest.mark.parametrize("motor_theta_ref2", [0.0, 5.0 * macros.D2R, 10.0 * macros.D2R])
@pytest.mark.parametrize("interrupt_fraction", [0.0, 0.25, 0.5, 0.75])
def test_stepper_motor_controller_interrupt(show_plots, motor_theta_ref1, motor_theta_ref2, interrupt_fraction):
    r"""
    **Validation Test Description**

    This unit test ensures that the stepper motor controller module correctly handles reference messages that interrupt
    an unfinished motor actuation sequence. The initial and reference motor angles are varied so that combinations of
    both positive and negative steps are taken. The time of step interruption is varied to ensure that once a step
    begins, it is completed regardless of when the interrupted message is written. Because the nominal unit test script
    for this module checked the module functionality for various motor step angles and reference angles that are not
    multiples of the motor step angle, the step angle, step time, and reference angles chosen in this script are
    set to simple values.

    **Test Parameters**

    Args:
        motor_theta_ref1 (float): [rad] Reference motor angle 1
        motor_theta_ref2 (float): [rad] Reference stepper motor angle 2
        interrupt_fraction (float): Specifies what fraction of a step is completed when the interrupted message is written

    **Description of Variables Being Tested**

    The module-computed number of required stepper motor steps for both simulation chunks are checked to match the
    number of motor steps computed in this script.

    """

    unit_task_name = "unitTask"
    unit_process_name = "TestProcess"
    unit_test_sim = SimulationBaseClass.SimBaseClass()
    test_process_rate = macros.sec2nano(0.1)
    test_proc = unit_test_sim.CreateNewProcess(unit_process_name)
    test_proc.addTask(unit_test_sim.CreateNewTask(unit_task_name, test_process_rate))

    # Create the stepperMotorController module
    motor_step_angle = 1.0 * macros.D2R  # [rad]
    motor_step_time = 1.0  # [s]
    motor_theta_init = 0.0  # [rad]
    motor_controller = stepperMotorController.StepperMotorController()
    motor_controller.modelTag = "stepperMotorController"
    motor_controller.setStepAngle(motor_step_angle)  # [rad]
    motor_controller.setStepTime(motor_step_time)  # [s]
    motor_controller.setThetaInit(motor_theta_init)  # [rad]
    unit_test_sim.AddModelToTask(unit_task_name, motor_controller)

    # Create the stepperMotorController input message
    hinged_rigid_body_message_data = messaging.HingedRigidBodyMsgPayload()
    hinged_rigid_body_message_data.theta = motor_theta_ref1  # [rad]
    hinged_rigid_body_message = messaging.HingedRigidBodyMsg().write(hinged_rigid_body_message_data)
    motor_controller.motorRefAngleInMsg.subscribeTo(hinged_rigid_body_message)

    # Set up data logging
    motor_step_command_log = motor_controller.motorStepCommandOutMsg.recorder(test_process_rate)
    unit_test_sim.AddModelToTask(unit_task_name, motor_step_command_log)

    # Calculate required number of steps to achieve the reference angle
    steps_commanded_truth1 = (motor_theta_ref1 - (np.ceil(motor_theta_init / motor_step_angle) * motor_step_angle)) / motor_step_angle

    # If the reference motor angle is not a multiple of the motor step angle, the number of steps calculated is not an
    # integer and it must be rounded to the nearest integer step
    lower_step_fraction = steps_commanded_truth1 - np.floor(steps_commanded_truth1)
    upper_step_fraction = np.ceil(steps_commanded_truth1) - steps_commanded_truth1
    if upper_step_fraction > lower_step_fraction:
        steps_commanded_truth1 = np.floor(steps_commanded_truth1)
    else:
        steps_commanded_truth1 = np.ceil(steps_commanded_truth1)

    # Compute the time required for the motor to actuate to the reference angle
    actuate_time1 = motor_step_time * np.abs(steps_commanded_truth1)  # [s]

    # Compute the simulation time for chunk 1/2
    # Set the simulation time to be half of the determined motor actuation time, plus a fraction of the time it takes
    # for the motor to complete a step so that the simulation is interrupted during a step
    sim_time1 = (actuate_time1 / 2) + (interrupt_fraction * motor_step_time)  # [s]

    # Run simulation chunk 1/2 (This simulation chunk ends at actuation interruption)
    unit_test_sim.InitializeSimulation()
    unit_test_sim.ConfigureStopTime(macros.sec2nano(sim_time1))
    unit_test_sim.ExecuteSimulation()

    # Create a new motor angle reference message (This message interrupts and overwrites the first reference message)
    hinged_rigid_body_message_data = messaging.HingedRigidBodyMsgPayload()
    hinged_rigid_body_message_data.theta = motor_theta_ref2  # [rad]
    hinged_rigid_body_message = messaging.HingedRigidBodyMsg().write(hinged_rigid_body_message_data, unit_test_sim.TotalSim.getCurrentNanos())
    motor_controller.motorRefAngleInMsg.subscribeTo(hinged_rigid_body_message)

    # Calculate number of steps to actuate from interrupted motor position to the second reference motor angle
    if steps_commanded_truth1 > 0:
        interrupted_motor_angle = motor_theta_init + ((sim_time1 / motor_step_time) * motor_step_angle)
        # Ensure the interrupted motor angle is set to the next multiple of the motor step angle
        # (If the motor is interrupted during a step)
        interrupted_motor_angle = np.ceil(interrupted_motor_angle / motor_step_angle) * motor_step_angle
    else:
        interrupted_motor_angle = motor_theta_init - ((sim_time1 / motor_step_time) * motor_step_angle)
        # Ensure the interrupted motor angle is set to the next multiple of the motor step angle
        # (If the motor is interrupted during a step)
        interrupted_motor_angle = np.floor(interrupted_motor_angle / motor_step_angle) * motor_step_angle

    steps_commanded_truth2 = (motor_theta_ref2 - interrupted_motor_angle) / motor_step_angle

    # If the reference motor angle is not a multiple of the step angle, the number of steps calculated is not an integer
    # and it must be rounded to the nearest integer step
    lower_step_fraction = steps_commanded_truth2 - np.floor(steps_commanded_truth2)
    upper_step_fraction = np.ceil(steps_commanded_truth2) - steps_commanded_truth2
    if upper_step_fraction > lower_step_fraction:
        steps_commanded_truth2 = np.floor(steps_commanded_truth2)
    else:
        steps_commanded_truth2 = np.ceil(steps_commanded_truth2)

    # Set the simulation time for chunk 2
    sim_time2 = motor_step_time * np.abs(steps_commanded_truth2)  # [s]

    # Run simulation chunk 2/2
    unit_test_sim.ConfigureStopTime(macros.sec2nano(sim_time1 + sim_time2 + 5.0))
    unit_test_sim.ExecuteSimulation()

    # Pull the logged motor step data
    steps_commanded = motor_step_command_log.stepsCommanded

    # Check that the correct number of steps was calculated for both simulation chunks
    accuracy = 1e-12
    np.testing.assert_allclose(steps_commanded[0],
                               steps_commanded_truth1,
                               atol=accuracy,
                               verbose=True)

    np.testing.assert_allclose(steps_commanded[-1],
                               steps_commanded_truth2,
                               atol=accuracy,
                               verbose=True)


if __name__ == "__main__":
    test_stepper_motor_controller_nominal(
         False,
         1.0 * macros.D2R,  # [rad] motorStepAngle
         1.0,  # [s] motorStepTime
         0.0,  # [rad] motorThetaInit
         10.0 * macros.D2R,  # [rad] motorThetaRef
    )
    test_stepper_motor_controller_invalid(
        False,
        270.0 * macros.D2R,  # [rad] motorThetaRef
    )
    test_stepper_motor_controller_interrupt(
        False,
        10.0 * macros.D2R,  # [rad] motorThetaRef1,
        5.0 * macros.D2R,  # [rad] motorThetaRef2
        0.0   # interruptFraction
    )
