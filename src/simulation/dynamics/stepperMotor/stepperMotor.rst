=============
Stepper Motor
=============

Introduction
------------
The stepper motor module simulates the actuation of a stepper motor. Given the initial motor angle :math:`\theta_0`, a
fixed motor step angle :math:`\Delta\theta`, a fixed motor step time :math:`\Delta t`, and an input message containing
an integer number of steps commanded, the motor states are computed at each time step and output from the module.
The motor states include the scalar motor angle :math:`\theta`, scalar angle rate :math:`\dot{\theta}`, scalar angular
acceleration :math:`\ddot{\theta}`, the current motor step count :math:`c_s` (within the active command, reset on each
new non-interrupting command), the absolute motor position :math:`p_s` in steps (cumulative net signed step count,
never reset; ``motorPosition`` in the output payload, with ``motorPosition * stepAngle`` ≈ :math:`\theta`), the number
of steps commanded to the motor :math:`n_s`, and a boolean ``isMotorMoving`` flag that is true while the motor is
actively actuating through a command and false when no actuation is in progress. The motor actuation through each
step of the command sequence is profiled using a bang-bang acceleration profile.

The module supports two types of incoming commands. A standard step command (``stopMotorCommand = false``) sets the
number of steps to actuate; if the motor is already moving, the new command interrupts the current actuation and the
motor finishes the current step before beginning to follow the new command. A stop command (``stopMotorCommand =
true``) halts the motor after the current step completes; because a stepper motor cannot stop actuating mid-step, the
motor reports ``isMotorMoving = true`` until the in-progress step finishes, then transitions to ``isMotorMoving =
false`` at a step-aligned angle.

Module Input/Output Messages
============================
The following table lists the module input and output messages.

.. list-table:: Module I/O Messages
    :widths: 25 25 50
    :header-rows: 1

    * - Msg Variable Name
      - Msg Type
      - Description
    * - motorStepCommandInMsg
      - :ref:`MotorStepCommandMsgPayload`
      - Input message containing the number of commanded motor steps (``stepsCommanded``) and a stop-request flag
        (``stopMotorCommand``). When ``stopMotorCommand`` is true the motor halts after completing its current step.
    * - stepperMotorOutMsg
      - :ref:`StepperMotorMsgPayload`
      - Output message containing the stepper motor states

Motor Step Actuation Profile
============================
The motor states are profiled identically for each step in the command sequence. Specifically, a bang-bang acceleration
profile is used to profile the motor states as a function of time. Given the motor step time :math:`\Delta t` and motor
step angle :math:`\Delta \theta` as fixed parameters, the module calculates the required acceleration
:math:`\ddot{\theta}_{\text{max}}` that must be applied during each motor step.

.. math::
    | \ddot{\theta}_{\text{max}} | = \frac{4 \Delta \theta}{\Delta t^2}

Note that the motor can take either positive or negative steps. For a forward actuation command (:math:`n_s > 0`), the
calculated acceleration is applied positively for the first half of the step and negatively during the second half of
the step. For a backward actuation command (:math:`n_s < 0`), the acceleration is applied negatively during the first
half of the step and positively during the second half of the step.

Given the initial time :math:`t_0`, the switch time :math:`t_s` where the acceleration is alternated and the final time
:math:`t_f` when the step is complete is determined as

.. math::
    t_s = t_0 + \frac{\Delta t}{2}

    t_f = t_0 + \Delta t

The other motor states can be kinematically profiled as a function of time by integrating the applied acceleration
profile. The equations used to profile each motor step are

.. math::
    \ddot{\theta}(t) =
    \begin{cases}
    \pm \ddot{\theta}_{\text{max}}
    & \text{if }
    t_0 \leq t < t_s
    \\
    \mp \ddot{\theta}_{\text{max}}
    & \text{if }
    t_s \leq t \leq t_f
    \\
    0
    & \text{if }
    t > t_f
    \end{cases}

    \dot{\theta}(t) =
    \begin{cases}
    \pm \ddot{\theta}_{\text{max}} (t - t_0)
    & \text{if }
    t_0 \leq t < t_s
    \\
    \mp \ddot{\theta}_{\text{max}} (t - t_f)
    & \text{if }
    t_s \leq t \leq t_f
    \\
    0
    & \text{if }
    t > t_f
    \end{cases}

    \theta(t) =
    \begin{cases}
    \frac{\pm \Delta \theta (t - t_0)^2}{2 (t_s - t_0)^2} + \theta_0
    & \text{if }
    t_0 \leq t < t_s
    \\
    \frac{\mp \Delta \theta (t - t_f)^2}{2 (t_s - t_f)^2} + \theta_{\text{ref}}
    & \text{if }
    t_s \leq t \leq t_f
    \\
    \theta_{\text{ref}}
    & \text{if }
    t > t_f
    \end{cases}

Note that the parameters :math:`t_0, t_s` and :math:`t_f` must be continually updated after each step is complete to
reflect the advancement of time. Doing so enables use of the above equations for each motor step. Specifically,
:math:`t_0` for the next step is anchored to the previous step's :math:`t_f` (rather than to the current sample time at
which the step-complete event is detected). This preserves the long-run motor rate of :math:`1 / \Delta t` when the
module update period does not divide :math:`\Delta t` evenly; otherwise the residual :math:`(t - t_f)` would be lost at
each step boundary and the effective step rate would drop below :math:`1 / \Delta t`.

.. important::
    If the motor actuation is interrupted by a new reference message while actuating through a step,
    the motor must finish actuating through the current step before it can begin following a new reference command.
    If the interrupting message is written when the motor is not in the midst of a step, the module resets the motor
    step count and immediately begins actuating to follow the new reference command.

Stop Command Handling
=====================
When an input message arrives with ``stopMotorCommand = true``, the module sets an internal ``pendingStop`` flag
instead of immediately resetting the actuation. The stop is then honored at the next step-boundary:

    - If the motor is **mid-step** when the stop arrives, the in-progress step runs to completion through the normal
      bang-bang profile. At the step boundary the module sets ``actuationComplete = true``, zeroes :math:`\dot\theta`
      and :math:`\ddot\theta`, and reports ``isMotorMoving = false`` on the next tick.
    - If the motor is **between steps** (``stepComplete = true``) when the stop is honored, the module halts before
      starting the next step.
    - If the motor is **already idle** (``actuationComplete = true``) when the stop arrives, the stop is a no-op —
      ``pendingStop`` is not set.
    - If a fresh non-stop command arrives before a pending stop is honored, ``pendingStop`` is cleared and the new
      command is processed normally (interrupting the current actuation if applicable).

Because the stop halts at a step boundary, :math:`\theta` lands on an integer multiple of :math:`\Delta\theta` from the
origin, ``motorPosition`` remains consistent with :math:`\theta` (``motorPosition * stepAngle`` ≈ :math:`\theta`), and
the algorithm consuming ``isMotorMoving`` sees the motor as settled only when it is physically at rest.

Module Functions
================
Below is a list of functions that this simulation module performs

    - Reads the incoming motor step command message (``stepsCommanded`` and ``stopMotorCommand``)
    - Computes the motor states as a function of time using a bang-bang acceleration profile
    - Tracks the absolute motor position (``motorPosition``) across commands
    - Writes the motor states to the module output message
    - Handles interruptions to motor actuation by resetting the motor actuation after the current step is complete
    - Honors stop commands by halting after the current step completes, preserving step-aligned :math:`\theta` and
      consistent ``motorPosition``

Module Assumptions and Limitations
==================================
    - The motor step angle and step time are fixed parameters (Cannot be negative)
    - The motor cannot stop actuating in the middle of a step
    - When the motor actuation is interrupted by a new reference command, the motor must complete its actuation through the current step before following the new command
    - Stop commands are likewise honored only at the next step boundary
    - The module update rate must be faster than or equal to the provided motor step time :math:`\Delta t`
    - The module update rate cannot be slower than the motor step rate, or the motor actuation cannot be resolved

Test Description and Success Criteria
=====================================
The unit tests for this module live in ``simulation/dynamics/stepperMotor/_UnitTest/test_stepperMotor.py``. The common
success criterion across all tests is that the motor states converge to independently computed reference values at
the end of each actuation sequence. Specifically, the motor angle, rate, acceleration, and step count are checked
to converge to the reference values at the end of each simulation chunk. The motor rate is zero at the completion of
each motor step, while the motor acceleration is nonzero at the instant of step completion and is therefore checked
one time step later, after the acceleration has been cleared.

Nominal Test
------------
``test_stepper_motor_nominal`` configures two actuation command segments separated by a 5-second rest period, with a
trailing 5-second rest for plot clarity. The initial motor angle, motor step angle, step time, and steps commanded
for each actuation sequence are varied so that the motor actuates both forwards and backwards. A zero-step command
is also exercised to verify the module correctly handles a no-op command.

Interruption Test
-----------------
``test_stepper_motor_interrupt`` verifies that the module correctly handles reference messages that interrupt an
unfinished motor actuation sequence. The first command is interrupted after half of its commanded steps have been
completed. The time the second command message is written is parameterized by an interruption factor specifying what
fraction of the next step has completed before the interrupt arrives. Interruption factors of 0 and 1 are included
to ensure the module correctly resets when the interrupt falls precisely on a step boundary. A 5-second trailing
rest is added for plot clarity.

Timestep Overshoot Test
-----------------------
``test_stepper_motor_timestep_overshoot`` verifies the motor correctly completes actuation when the simulation
timestep does not divide evenly into the motor step time. In this scenario the simulation time overshoots the step
completion time :math:`t_f`. The test guards against a regression in which the leftover :math:`(t - t_f)` residual
was dropped at each step boundary, causing the motor to either never reach step-complete or to fall behind the
expected step rate of :math:`1 / \Delta t`.

Single-Step Edge Case Test
--------------------------
``test_stepper_motor_single_step`` verifies the motor correctly actuates for a single-step command in both forward
(``stepsCommanded = +1``) and backward (``stepsCommanded = -1``) directions — an edge case where the motor must
complete exactly one step and immediately come to rest.

Rapid Command Interruption Test
-------------------------------
``test_stepper_motor_rapid_commands`` verifies the module's behavior under multiple back-to-back command interrupts.
Three commands are sent in rapid succession, each interrupting the previous after only 2 steps have completed. The
test checks that the motor angle, rate, and step count converge to the expected values after the final command
completes.
