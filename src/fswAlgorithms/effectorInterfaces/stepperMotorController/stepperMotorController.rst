==============================
Stepper Motor Controller
==============================

.. list-table::
   :widths: 5 40 10 10
   :header-rows: 0

   * - **Rev**
     - **Change Description**
     - **By**
     - **Date**
   * - 1.0
     - Initial Draft
     - L. Kiner
     - 20250723


====================
Module Description
====================

Introduction
============
This stepper motor flight software module computes the number of motor steps required to actuate a stepper motor from
its current angle to a specified reference motor angle. Each time a new motor reference message is written
to this module, the required motor steps commanded to achieve the incoming reference angle are computed, updated, and
output from the module. The module outputs zero steps commanded if the reference motor angle is outside of the
motor actuation limits that are configurable by the user. Finally, this module also includes logic for handling incoming
reference commands that interrupt an unfinished motor actuation sequence. Because the stepper motor is unable to stop
actuating during a step, it must finish actuating through the current step before it can begin following
a new reference command.

Module Input/Output Messages
============================
The following table lists the module input and output messages.

.. list-table:: Module I/O Messages
    :widths: 25 25 50
    :header-rows: 1

    * - Msg Variable Name
      - Msg Type
      - Description
    * - motorRefAngleInMsg
      - :ref:`HingedRigidBodyMsgPayload`
      - Input message containing the stepper motor reference angle message
    * - motorStepCommandOutMsg
      - :ref:`MotorStepCommandMsgPayload`
      - Output message containing the number of commanded motor steps

Algorithm Flow
==============
Each time a new motor angle input reference message is read, this module computes the required number of
integer motor steps :math:`n_s` to actuate to a reachable motor angle nearest to the specified
reference angle :math:`\theta_{\text{ref}`. The reference angle is not reachable if it is not a multiple
of the fixed motor step angle :math:`\Delta\theta`.

The integer number of motor steps commanded is calculated as

.. math::

   n_s =
   \begin{cases}
   \left\lfloor \dfrac{\theta_{\text{ref}} - \theta}{\Delta\theta} \right\rfloor
   & \text{if }
   \left\lceil \dfrac{\theta_{\text{ref}} - \theta}{\Delta\theta} \right\rceil
   - \dfrac{\theta_{\text{ref}} - \theta}{\Delta\theta}
   >
   \dfrac{\theta_{\text{ref}} - \theta}{\Delta\theta}
   - \left\lfloor \dfrac{\theta_{\text{ref}} - \theta}{\Delta\theta} \right\rfloor \\
   \left\lceil \dfrac{\theta_{\text{ref}} - \theta}{\Delta\theta} \right\rceil
   & \text{if }
   \left\lceil \dfrac{\theta_{\text{ref}} - \theta}{\Delta\theta} \right\rceil
   - \dfrac{\theta_{\text{ref}} - \theta}{\Delta\theta}
   <
   \dfrac{\theta_{\text{ref}} - \theta}{\Delta\theta}
   - \left\lfloor \dfrac{\theta_{\text{ref}} - \theta}{\Delta\theta} \right\rfloor
   \end{cases}

where :math:`\theta` is the current motor angle.

.. important::
    If the motor actuation is interrupted by a new reference message while actuating through a step,
    the current motor angle is updated to the next multiple of the motor step angle. This is because
    the motor must finish actuating through the current step before it can begin following a new reference command.

For interrupting reference messages, the motor angle :math:`\theta` in the above equation is updated to the following
in order to compute the number of steps commanded

.. math::
    \theta =
    \begin{cases}
    \left\lfloor \dfrac{\theta}{\Delta\theta} \right\rfloor \Delta\theta
    & \text{if }
    \theta
    <
    0 \\
    \left\lceil \dfrac{\theta}{\Delta\theta} \right\rceil \Delta\theta
    & \text{if }
    \theta
    >
    0
    \end{cases}

The motor reference angle is updated to the reachable value after the number of steps is determined

.. math::
    \theta_{\text{ref}} = \theta + n_s \Delta \theta

The module outputs the number of steps commanded only after a new motor angle reference input message is read.
The motor step count is set to zero after a new reference command is read by the module.

This module also tracks the motor actuation in time for each command sequence by updating the
current motor angle and step count at each time step.

.. important::
    While this module tracks the motor step count and motor angle, these are meant to be completely internal to the
    controller module and do not sufficiently represent the true motor parameters which are tracked in the stepper motor
    simulation module. This module roughly tracks the motor angle in order to determine the number of steps
    commanded. This module linearly tracks the motor angle for each step, which is not representative of the true
    stepper motor profile for each step.

With this in mind, the motor angle is updated in this module as

.. math::
    \theta = \theta_0 + \frac{\Delta t_{\text{sim}}}{\Delta t_{\text{step}}} \Delta \theta

where :math:`\Delta t_{\text{step}}` is the motor step time and :math:`\Delta t_{\text{sim}}` is the time elapsed
since the last motor reference input message was written (:math:`\Delta t_{\text{sim}} = t - t_{\text{prev}}`)

The motor step count is updated as

.. math::
    c_s = \pm \left\lfloor \frac{\Delta t_{\text{sim}}}{\Delta t_{\text{step}}} \right\rfloor

Controller Functions
====================
Below is a list of functions that this flight software module performs

    - Reads the incoming motor reference angle message
    - Computes the number of motor steps required to reach the reference angle
    - Writes the output message for number of motor steps commanded
    - Handles interruptions to motor actuation by resetting the motor actuation after the current step is complete
    - Outputs zero steps commanded if the reference motor angle is outside of the motor actuation limits

Controller Assumptions and Limitations
======================================
    - The motor step angle and step time are fixed parameters
    - The motor cannot stop actuating in the middle of a step
    - When motor actuation is interrupted by a new reference command, the motor completes the current step only, ignoring all other commanded steps and re-computes the new number of steps commanded to reach the new reference
    - The motor has user-configurable upper and lower actuation bounds. The default bounds are :math:`(-2\pi, 2\pi)`

Test Description and Success Criteria
=====================================
There are three tests for this module. The tests are located in ``fswAlgorithms/effectorInterfaces/stepperMotorController/_UnitTest/test_stepperMotorController.py``
The first test is a nominal test named ``test_stepperMotorController_nominal``. The second test named
``test_stepperMotorController_invalid`` checks that zero steps commanded are output if the reference motor angle is
outside of the specified motor actuation bounds. The third test is an interruption test named ``test_stepperMotorController_interrupt``.
The success criteria for all tests is that the commanded number of steps computed and output from the controller module
matches the value determined in the test.

Note that while this module internally tracks the current motor angle and motor step count, these parameters are
not checked in the unit test. As previously mentioned, the stepper motor simulation module located in
``simulation/dynamics/stepperMotor`` precisely tracks these parameters.

Nominal Test
------------
The nominal unit test ensures that the stepper motor controller module correctly determines the number of steps required to
actuate from an initial angle to a final reference angle. The initial and reference motor angles are varied so that
both positive and negative steps are required in this test. It must be noted that the motor angles are discretized
by a constant motor step angle; therefore the motor cannot simply actuate to any desired angle.
The reference motor angles are chosen in this test so that several cases require the reference to be adjusted
to the nearest multiple of the motor step angle. In other words, this test checks cases where the exact number of
motor steps required to reach the reference exactly is not an integer. For these cases, the final result for the
number of commanded motor steps is rounded to the nearest integer step and the corresponding motor reference
angle is updated to the reachable value.

Invalid Test
------------
The invalid unit test ensures that the stepper motor controller module outputs zero steps commanded when the
reference motor angle is outside the motor actuation bounds.

Interruption Test
-----------------
The interruption unit test ensures that the stepper motor controller module correctly handles reference messages that
interrupt an unfinished motor actuation sequence. The initial and reference motor angles are varied so that combinations
of both positive and negative steps are taken. The time of step interruption is varied to ensure that once a step
begins, it is completed regardless of when the interrupted message is written. Because the nominal unit test script
checks the module functionality for various motor step angles and reference angles that are not
multiples of the motor step angle, the step angle, step time, and reference angles chosen in this script are
set to simple values.
