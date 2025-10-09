Executive Summary
-----------------

A thruster force message is read in and converted to a thruster on-time output message. The module ensures the requested on-time is at least as large as the thruster's minimum on time.  If not then the on-time is zeroed, but the unimplemented thrust time is kept as a remainder calculation.  If these add up to reach the minimum on time, then a thruster pulse is requested.  If the thruster on time is larger than the control period, then an on-time that is 1.1 times the control period is requested. More information can be found in the
:download:`PDF Description </../../src/fswAlgorithms/effectorInterfaces/thrFiringRemainder/_Documentation/Basilisk-thrFiringRemainder-2023-08-07.pdf>`.
The paper `Steady-State Attitude and Control Effort Sensitivity Analysis of Discretized Thruster Implementations <https://doi.org/10.2514/1.A33709>`__ includes a detailed discussion on the Remainder Trigger algorithm and compares it to other thruster firing methods.


Message Connection Descriptions
-------------------------------
The following table lists all the module input and output messages.  The module msg connection is set by the
user from python.  The msg type contains a link to the message structure definition, while the description
provides information on what this message is used for.

.. _ModuleIO_ThrFiringRemainder:
.. figure:: /../../src/fswAlgorithms/effectorInterfaces/thrFiringRemainder/_Documentation/Images/moduleImgThrFiringRemainder.svg
    :align: center

    Figure 1: ``rwNullSpace()`` Module I/O Illustration


.. list-table:: Module I/O Messages
    :widths: 25 25 50
    :header-rows: 1

    * - Msg Variable Name
      - Msg Type
      - Description
    * - thrForceInMsg
      - :ref:`THRArrayCmdForceMsgPayload`
      - thruster force input message
    * - onTimeOutMsg
      - :ref:`THRArrayOnTimeCmdMsgPayload`
      - thruster on-time output message
    * - thrConfInMsg
      - :ref:`THRArrayConfigMsgPayload`
      - Thruster array configuration input message


Module Description
==================

This module implements a remainder tracking thruster firing logic. More
details can be found in Reference.

Module Input and Output Messages
--------------------------------

As illustrated in Figure `[fig:moduleImg] <#fig:moduleImg>`__, the
module reads in two messages. One message contains the thruster
configuration message from which the maximum thrust force value for each
thruster is extracted and stored in the module. This message is only
read in on ``reset()``.

The second message reads in an array of requested thruster force values
with every ``Update()`` function call. These force values :math:`F_{i}`
can be positive if on-pulsing is requested, or negative if off-pulsing
is required. On-pulsing is used to achieve an attitude control torque
onto the spacecraft by turning on a thruster. Off-pulsing assumes a
thruster is nominally on, such as with doing an extended orbit
correction maneuver, and the attitude control is achieved by doing
periodic off-pulsing.

The output of the module is a message containing an array of thruster
on-time requests. If these on-times are larger than the control period,
then the thruster remains on only for the control period upon which the
on-criteria is reevaluated.

reset() Functionality
---------------------

- The control period is dynamically evaluated in the module by comparing
  the current time with the prior call time. In ``reset()`` the
  ``prevCallTime`` variable is reset to 0.

- The thruster configuration message is read in and the number of
  thrusters is stored in the module variable ``numThrusters``. The
  maximum force per thruster is stored in ``maxThrust``.

- The on-time pulse remainder variable is reset for each thruster back
  to 0.0.

Update() Functionality
----------------------

The goal of the ``update()`` method is to read in the current attitude
control thruster force message and map these into a thruster on-time
output message. Let :math:`\Delta t_{\text{min}}` be the minimum on-time
that can be implemented with a thruster. If the requested on-time is
less than :math:`\Delta t_{\text{min}}`, then the requested thruster
on-time is clipped to zero. In the following algorithm unimplemented
fractional on-times less than :math:`\Delta t_{\text{min}}` are tracked
and accumulated, providing additional pointing accuracy. For example, if
the minimum on-time is 20 milli-seconds, an on-time algorithm without
remainder calculation would create a deadband about the 20 milli-second
control request. With the remainder logic, if 5 milli-second on-time
requests are computed, these are accumulated such that every
4\ :math:`^{\text{th}}` control step a 20 milli-second burn is
requested. This reduces the deadband behavior of the thruster and
achieves better pointing. In this example the 5 milli-second
un-implemented on-times are accumulated in the variable
:math:`\Delta t_{\text{partial}}`.

If the ``update()`` method is called for the first time after reset,
then there is no knowledge of the control period :math:`\Delta t`. In
this case the control period is set to 2 seconds, unless the module
parameter ``defaultControlPeiod`` is set to a different value. If this
is a repeated call of the ``update()`` method then the control period
:math:`\Delta t` is evaluated by differencing the current time with the
prior call time. Next a loop goes over each installed thruster to map
the requested force :math:`F_{i}` into an on-time :math:`t_{i}`. The
following logic is used.

- If off-pulsing is used then :math:`F_{i}\le 0` and we set

  .. math:: F_{i} += F_{\text{max}}

  \ to a reduced thrust to achieve the negative :math:`F_{i}` force.

- Next, if :math:`F_{i} < 0` then it set to be equal to zero. This can
  occur if an off-pulsing request is larger than the maximum thruster
  force magnitude :math:`F_{\text{max}}`.

- The nominal thruster on-time is computed using

  .. math:: t_{i}  = \dfrac{F_{i}}{F_{\text{max}}} \Delta t

  .

- If there un-implemented on-time requested
  :math:`\Delta t_{\text{partial}}` from earlier ``update()`` method
  calls, these are added to the current on-time request using

  .. math:: t_{i} += \Delta t_{\text{partial}}

  \ After this step the variable :math:`\Delta t_{\text{partial}}` is
  reset to 0 as the remainder calculation is stored in the on-time
  variable :math:`t_{i}`.

- If :math:`t_{i} < \Delta t_{\text{partial}}` then on-time request is
  set to zero and the remained is set to
  :math:`\Delta t_{\text{partial}} = t_{i}`

- If :math:`t_{i} > \Delta` then the requested force is larger than
  :math:`F_{\text{max}}` and the control is saturated. In this case the
  on-time is set to 1.1\ :math:`\Delta t` such that the thruster remains
  on through the control period.

- The final step is to store the thruster on-time into and write out
  this output message

Module Functions
================

- **Read in thruster configuration message**: This is used to determine
  the number of installed thrusters and what the maximum force is for
  each.

- **Convert thruster force requested into an on-time request**: Knowing
  how strong the thruster is, the on-time is scaled such that the
  effectively applied force is equal to the requested force.

Module Assumptions and Limitations
==================================

The module assumes that the incoming forces :math:`F_{i}` can be both
positive or negative, depending if an on- or off-pulsing mode is being
implemented. The particular mode is set through ``baseThrustState``.
