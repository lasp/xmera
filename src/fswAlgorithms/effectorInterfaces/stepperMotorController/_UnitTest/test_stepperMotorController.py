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

@pytest.mark.parametrize("motorStepAngle", [0.008 * macros.D2R, 0.01 * macros.D2R, 0.5 * macros.D2R])
@pytest.mark.parametrize("motorStepTime", [0.008, 0.1, 0.5])
@pytest.mark.parametrize("motorThetaInit", [-5.0 * macros.D2R, 0.0, 60.0 * macros.D2R])
@pytest.mark.parametrize("motorThetaRef", [0.0, 10.6 * macros.D2R, 60.0051 * macros.D2R])
def test_stepperMotorController_nominal(show_plots, motorStepAngle, motorStepTime, motorThetaInit, motorThetaRef):
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
        motorStepAngle (float): [rad] Step angle the motor rotates through for a single step (constant)
        motorStepTime (float): [sec] Time required for the motor to actuate through a single step (constant)
        motorThetaInit (float): [rad] Initial stepper motor angle
        motorThetaRef (float): [rad] Reference stepper motor angle

    **Description of Variables Being Tested**

    The module-computed number of required stepper motor steps is checked to match the number of motor steps
    computed in this script.

    """

    unitTaskName = "unitTask"
    unitProcessName = "TestProcess"
    unitTestSim = SimulationBaseClass.SimBaseClass()
    testProcessRate = macros.sec2nano(motorStepTime)
    testProc = unitTestSim.CreateNewProcess(unitProcessName)
    testProc.addTask(unitTestSim.CreateNewTask(unitTaskName, testProcessRate))

    # Create the stepperMotorController module
    motorController = stepperMotorController.StepperMotorController()
    motorController.modelTag = "stepperMotorController"
    motorController.setStepAngle(motorStepAngle)  # [rad]
    motorController.setStepTime(motorStepTime)  # [s]
    motorController.setThetaInit(motorThetaInit)  # [rad]
    unitTestSim.AddModelToTask(unitTaskName, motorController)

    # Create the stepperMotorController input message
    HingedRigidBodyMessageData = messaging.HingedRigidBodyMsgPayload()
    HingedRigidBodyMessageData.theta = motorThetaRef  # [rad]
    HingedRigidBodyMessage = messaging.HingedRigidBodyMsg().write(HingedRigidBodyMessageData)
    motorController.motorRefAngleInMsg.subscribeTo(HingedRigidBodyMessage)

    # Set up data logging
    motorStepCommandLog = motorController.motorStepCommandOutMsg.recorder(testProcessRate)
    unitTestSim.AddModelToTask(unitTaskName, motorStepCommandLog)

    # Calculate required number of motor steps to achieve the reference angle
    if (motorThetaInit > 0):
        stepsCommandedTruth = (motorThetaRef - (np.ceil(motorThetaInit/motorStepAngle)*motorStepAngle)) / motorStepAngle
    else:
        stepsCommandedTruth = (motorThetaRef - (np.floor(motorThetaInit/motorStepAngle)*motorStepAngle)) / motorStepAngle

    # If the reference motor angle is not a multiple of the motor step angle, the number of steps calculated is not an
    # integer and it must be rounded to the nearest integer step
    lowerStepFraction = stepsCommandedTruth - np.floor(stepsCommandedTruth)
    upperStepFraction = np.ceil(stepsCommandedTruth) - stepsCommandedTruth
    if (upperStepFraction > lowerStepFraction):
        stepsCommandedTruth = np.floor(stepsCommandedTruth)
    else:
        stepsCommandedTruth = np.ceil(stepsCommandedTruth)

    # Compute the time required for the motor to actuate to the reference angle
    actuateTime = motorStepTime * np.abs(stepsCommandedTruth)  # [s]

    # Run the simulation
    unitTestSim.InitializeSimulation()
    unitTestSim.ConfigureStopTime(macros.sec2nano(actuateTime))
    unitTestSim.ExecuteSimulation()

    # Pull the logged motor step data
    stepsCommandedSim = motorStepCommandLog.stepsCommanded

    # Check that the correct number of steps was calculated
    accuracy = 1e-12
    np.testing.assert_allclose(stepsCommandedSim[0],
                               stepsCommandedTruth,
                               atol=accuracy,
                               verbose=True)


@pytest.mark.parametrize("motorThetaRef", [-10.0, 275.0])
def test_stepperMotorController_invalid(show_plots, motorThetaRef):
    r"""
    **Validation Test Description**

    This unit test ensures that the stepper motor controller module correctly outputs zero steps commanded when the
    reference motor angle is outside the motor actuation bounds.

    **Test Parameters**

    Args:
        motorThetaRef (float): [rad] Reference stepper motor angle

    **Description of Variables Being Tested**

    The module-computed number of required stepper motor steps is checked to be zero in this test.

    """

    unitTaskName = "unitTask"
    unitProcessName = "TestProcess"
    unitTestSim = SimulationBaseClass.SimBaseClass()
    testProcessRate = macros.sec2nano(1.0)
    testProc = unitTestSim.CreateNewProcess(unitProcessName)
    testProc.addTask(unitTestSim.CreateNewTask(unitTaskName, testProcessRate))

    # Create the stepperMotorController module
    motorController = stepperMotorController.StepperMotorController()
    motorController.modelTag = "stepperMotorController"
    motorController.setThetaMin(180.0 * macros.D2R)  # [rad]
    motorController.setThetaMin(0.0)  # [rad]
    unitTestSim.AddModelToTask(unitTaskName, motorController)

    # Create the stepperMotorController input message
    HingedRigidBodyMessageData = messaging.HingedRigidBodyMsgPayload()
    HingedRigidBodyMessageData.theta = motorThetaRef  # [rad]
    HingedRigidBodyMessage = messaging.HingedRigidBodyMsg().write(HingedRigidBodyMessageData)
    motorController.motorRefAngleInMsg.subscribeTo(HingedRigidBodyMessage)

    # Set up data logging
    motorStepCommandLog = motorController.motorStepCommandOutMsg.recorder(testProcessRate)
    unitTestSim.AddModelToTask(unitTaskName, motorStepCommandLog)

    # Run the simulation
    unitTestSim.InitializeSimulation()
    unitTestSim.ConfigureStopTime(macros.sec2nano(3.0))
    unitTestSim.ExecuteSimulation()

    # Pull the logged motor step data
    stepsCommandedSim = motorStepCommandLog.stepsCommanded

    # Check that the correct number of steps was calculated
    accuracy = 1e-12
    np.testing.assert_allclose(stepsCommandedSim[0],
                               0,
                               atol=accuracy,
                               verbose=True)


@pytest.mark.parametrize("motorThetaRef1", [-10.0 * macros.D2R, 10.0 * macros.D2R])
@pytest.mark.parametrize("motorThetaRef2", [0.0, 5.0 * macros.D2R, 10.0 * macros.D2R])
@pytest.mark.parametrize("interruptFraction", [0.0, 0.25, 0.5, 0.75])
def test_stepperMotorController_interrupt(show_plots, motorThetaRef1, motorThetaRef2, interruptFraction):
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
        motorThetaRef1 (float): [rad] Reference motor angle 1
        motorThetaRef2 (float): [rad] Reference stepper motor angle 2
        interruptFraction (float): Specifies what fraction of a step is completed when the interrupted message is written

    **Description of Variables Being Tested**

    The module-computed number of required stepper motor steps for both simulation chunks are checked to match the
    number of motor steps computed in this script.

    """

    unitTaskName = "unitTask"
    unitProcessName = "TestProcess"
    unitTestSim = SimulationBaseClass.SimBaseClass()
    testProcessRate = macros.sec2nano(0.1)
    testProc = unitTestSim.CreateNewProcess(unitProcessName)
    testProc.addTask(unitTestSim.CreateNewTask(unitTaskName, testProcessRate))

    # Create the stepperMotorController module
    motorStepAngle = 1.0 * macros.D2R  # [rad]
    motorStepTime = 1.0  # [s]
    motorThetaInit = 0.0  # [rad]
    motorController = stepperMotorController.StepperMotorController()
    motorController.modelTag = "stepperMotorController"
    motorController.setStepAngle(motorStepAngle)  # [rad]
    motorController.setStepTime(motorStepTime)  # [s]
    motorController.setThetaInit(motorThetaInit)  # [rad]
    unitTestSim.AddModelToTask(unitTaskName, motorController)

    # Create the stepperMotorController input message
    HingedRigidBodyMessageData = messaging.HingedRigidBodyMsgPayload()
    HingedRigidBodyMessageData.theta = motorThetaRef1  # [rad]
    HingedRigidBodyMessage = messaging.HingedRigidBodyMsg().write(HingedRigidBodyMessageData)
    motorController.motorRefAngleInMsg.subscribeTo(HingedRigidBodyMessage)

    # Set up data logging
    motorStepCommandLog = motorController.motorStepCommandOutMsg.recorder(testProcessRate)
    unitTestSim.AddModelToTask(unitTaskName, motorStepCommandLog)

    # Calculate required number of steps to achieve the reference angle
    stepsCommandedTruth1 = (motorThetaRef1 - (np.ceil(motorThetaInit/motorStepAngle)*motorStepAngle)) / motorStepAngle

    # If the reference motor angle is not a multiple of the motor step angle, the number of steps calculated is not an
    # integer and it must be rounded to the nearest integer step
    lowerStepFraction = stepsCommandedTruth1 - np.floor(stepsCommandedTruth1)
    upperStepFraction = np.ceil(stepsCommandedTruth1) - stepsCommandedTruth1
    if (upperStepFraction > lowerStepFraction):
        stepsCommandedTruth1 = np.floor(stepsCommandedTruth1)
    else:
        stepsCommandedTruth1 = np.ceil(stepsCommandedTruth1)

    # Compute the time required for the motor to actuate to the reference angle
    actuateTime1 = motorStepTime * np.abs(stepsCommandedTruth1)  # [s]

    # Compute the simulation time for chunk 1/2
    # Set the simulation time to be half of the determined motor actuation time, plus a fraction of the time it takes
    # for the motor to complete a step so that the simulation is interrupted during a step
    simTime1 = (actuateTime1 / 2) + (interruptFraction * motorStepTime)  # [s]

    # Run simulation chunk 1/2 (This simulation chunk ends at actuation interruption)
    unitTestSim.InitializeSimulation()
    unitTestSim.ConfigureStopTime(macros.sec2nano(simTime1))
    unitTestSim.ExecuteSimulation()

    # Create a new motor angle reference message (This message interrupts and overwrites the first reference message)
    HingedRigidBodyMessageData = messaging.HingedRigidBodyMsgPayload()
    HingedRigidBodyMessageData.theta = motorThetaRef2  # [rad]
    HingedRigidBodyMessage = messaging.HingedRigidBodyMsg().write(HingedRigidBodyMessageData, unitTestSim.TotalSim.getCurrentNanos())
    motorController.motorRefAngleInMsg.subscribeTo(HingedRigidBodyMessage)

    # Calculate number of steps to actuate from interrupted motor position to the second reference motor angle
    if (stepsCommandedTruth1 > 0):
        interruptedMotorAngle = motorThetaInit + ((simTime1 / motorStepTime) * motorStepAngle)
        # Ensure the interrupted motor angle is set to the next multiple of the motor step angle
        # (If the motor is interrupted during a step)
        interruptedMotorAngle = np.ceil(interruptedMotorAngle / motorStepAngle) * motorStepAngle
    else:
        interruptedMotorAngle = motorThetaInit - ((simTime1 / motorStepTime) * motorStepAngle)
        # Ensure the interrupted motor angle is set to the next multiple of the motor step angle
        # (If the motor is interrupted during a step)
        interruptedMotorAngle = np.floor(interruptedMotorAngle / motorStepAngle) * motorStepAngle

    stepsCommandedTruth2 = (motorThetaRef2 - interruptedMotorAngle) / motorStepAngle

    # If the reference motor angle is not a multiple of the step angle, the number of steps calculated is not an integer
    # and it must be rounded to the nearest integer step
    lowerStepFraction = stepsCommandedTruth2 - np.floor(stepsCommandedTruth2)
    upperStepFraction = np.ceil(stepsCommandedTruth2) - stepsCommandedTruth2
    if (upperStepFraction > lowerStepFraction):
        stepsCommandedTruth2 = np.floor(stepsCommandedTruth2)
    else:
        stepsCommandedTruth2 = np.ceil(stepsCommandedTruth2)

    # Set the simulation time for chunk 2
    simTime2 = motorStepTime * np.abs(stepsCommandedTruth2)  # [s]

    # Run simulation chunk 2/2
    unitTestSim.ConfigureStopTime(macros.sec2nano(simTime1 + simTime2 + 5.0))
    unitTestSim.ExecuteSimulation()

    # Pull the logged motor step data
    stepsCommanded = motorStepCommandLog.stepsCommanded

    # Check that the correct number of steps was calculated for both simulation chunks
    accuracy = 1e-12
    np.testing.assert_allclose(stepsCommanded[0],
                               stepsCommandedTruth1,
                               atol=accuracy,
                               verbose=True)

    np.testing.assert_allclose(stepsCommanded[-1],
                               stepsCommandedTruth2,
                               atol=accuracy,
                               verbose=True)


if __name__ == "__main__":
    test_stepperMotorController_nominal(
         False,
         1.0 * macros.D2R,  # [rad] motorStepAngle
         1.0,  # [s] motorStepTime
         0.0,  # [rad] motorThetaInit
         10.0 * macros.D2R,  # [rad] motorThetaRef
    )
    test_stepperMotorController_invalid(
        False,
        270.0 * macros.D2R,  # [rad] motorThetaRef
    )
    test_stepperMotorController_interrupt(
        False,
        10.0 * macros.D2R,  # [rad] motorThetaRef1,
        5.0 * macros.D2R,  # [rad] motorThetaRef2
        0.0   # interruptFraction
    )
