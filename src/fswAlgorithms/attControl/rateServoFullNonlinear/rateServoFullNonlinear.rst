Executive Summary
-----------------

This module implements a nonlinear rate servo control uses the attitude steering message and determine the ADCS control
torque vector.

Message Connection Descriptions
-------------------------------
The following table lists all the module input and output messages.  The module msg connection is set by the
user from python.  The msg type contains a link to the message structure definition, while the description
provides information on what this message is used for.

.. list-table:: Module I/O Messages
    :widths: 25 25 50
    :header-rows: 1

    * - Msg Variable Name
      - Msg Type
      - Description
    * - cmdTorqueOutMsg
      - :ref:`CmdTorqueBodyMsgPayload`
      - commanded torque output message
    * - guidInMsg
      - :ref:`AttGuidMsgPayload`
      - attitude guidance input message
    * - vehConfigInMsg
      - :ref:`VehicleConfigMsgPayload`
      - vehicle configuration input message
    * - rwSpeedsInMsg
      - :ref:`RWSpeedMsgPayload`
      - (optional) RW speed input message
    * - rwAvailInMsg
      - :ref:`RWAvailabilityMsgPayload`
      - (optional) RW availability input message
    * - rwParamsInMsg
      - :ref:`RWArrayConfigMsgPayload`
      - (optional) RW configuration parameter input message
    * - rateSteeringInMsg
      - :ref:`RateCmdMsgPayload`
      - commanded rate input message

Overview
========

This module computes a commanded control torque vector :math:`\mathbf L_r`
using a rate based steering law that drives a body frame
:math:`{\mathcal B} :\{ \hat{\mathbf B}_{1}, \hat{\mathbf B}_{2}, \hat{\mathbf B}_{3}\}`
towards a time varying reference frame
:math:`{\mathcal R} :\{ \hat{\mathbf R}_{1}, \hat{\mathbf R}_{2}, \hat{\mathbf R}_{3}\}`,
based on a desired reference frame
:math:`{\mathcal B*} :\{ \hat{\mathbf B}}_{1}, \hat{\mathbf B}}_{2}, \hat{\mathbf B}}_{3}\}`
(the desired body frame from the kinematic steering law).

The module input messages and output message are illustrated in
Figure `1 <#fig:moduleImg>`__. The output message is a body-frame
control torque vector that is outlined in section 3, with
:math:`\mathbf L_r` specifically computed in equation
`[eq:MS:39] <#eq:MS:39>`__. The required attitude guidance message
contains both attitude tracking error rates as well as reference frame
rates. This message is read in with every update cycle. The vehicle
configuration message is only read in on reset and contains the
spacecraft inertia tensor about the vehicle’s center of mass location.
The commanded body rates are read in from the steering module output
message.

The reaction wheel (RW) configuration message is optional. If the
message name is specified, then the RW message is read in. If the
optional RW availability message is present, then the control will only
use the RWs that are marked available.

The servo rate feedback control can compensate for Reaction Wheel (RW)
gyroscopic effects as well. This is an optional input message where the
RW configuration array message contains the RW spin axis
:math:`\hat{g}_{s,i}` information and the RW polar inertia about the
spin axis IWs,i . This is only read in on reset. The RW speed message
contains the RW speed :math:`\Omega_i` and is read in every time step.
The optional RW availability message can be used to include or not
include RWs in the MRP feedback. This allows the module to selectively
turn off some RWs. The default is that all RWs are operational and are
included.

Initialization
==============

Simply call the module reset function prior to using this control
module. This will reset the prior function call time variable, and reset
the rotational rate error integral measure. The control update period
:math:`\Delta t` is evaluated automatically.

Steering Law Goals
==================

This technical note develops a rate based steering law that drives a
body frame
:math:`{\mathcal B} :\{ \hat{\mathbf B}_{1}, \hat{\mathbf B}_{2}, \hat{\mathbf B}_{3}\}`
towards a time varying reference frame
:math:`{\mathcal R} :\{ \hat{\mathbf R}_{1}, \hat{\mathbf R}_{2}, \hat{\mathbf R}_{3}\}`.
The inertial frame is given by
:math:`{\mathcal N} :\{ \hat{\mathbf N}_{1}, \hat{\mathbf N}_{2}, \hat{\mathbf N}_{3}\}`.
The RW coordinate frame is given by
:math:`\mathcal{W_{i}}:\{ \hat{\mathbf g}_{s_{i}}, \hat{\mathbf g}_{t_{i}}, \hat{\mathbf g}_{g_{i}} \}`.
Using MRPs, the overall control goal is

.. math::

   \label{eq:MS:1}
       \mathbf\sigma_{\mathcal{B}/\mathcal{R}} \rightarrow 0

The reference frame orientation
:math:`\mathbf \sigma_{\mathcal{R}/\mathcal{N}}`, angular velocity
:math:`\mathbf\omega_{\mathcal{R}/\mathcal{N}}` and inertial angular
acceleration :math:`\dot{\mathbf \omega}_{\mathcal{R}/\mathcal{N}}` are
assumed to be known.

The rotational equations of motion of a rigid spacecraft with :math:`N`
Reaction Wheels (RWs) attached are given by:raw-latex:`\cite{schaub}`

.. math::

   \label{eq:MS:2}
       [I_{RW}] \dot{\mathbf \omega} = - [\tilde{\mathbf \omega}] \left(
       [I_{RW}] \mathbf\omega + [G_{s}] \mathbf h_{s}
       \right) - [G_{s}] \mathbf u_{s} + \mathbf L

where the inertia tensor :math:`[I_{RW}]` is defined as

.. math::

   \label{eq:MS:3}
       [I_{RW}] = [I_{s}] + \sum_{i=1}^{N} \left (J_{t_{i}} \hat{\mathbf g}_{t_{i}} \hat{\mathbf g}_{t_{i}}^{T} + J_{g_{i}} \hat{\mathbf g}_{g_{i}} \hat{\mathbf g}_{g_{i}}^{T}
       \right)

The spacecraft inertial without the :math:`N` RWs is :math:`[I_{s}]`,
while :math:`J_{s_{i}}`, :math:`J_{t_{i}}` and :math:`J_{g_{i}}` are the
RW inertias about the body fixed RW axis :math:`\hat{\mathbf g}_{s_{i}}` (RW
spin axis), :math:`\hat{\mathbf g}_{t_{i}}` and :math:`\hat{\mathbf g}_{g_{i}}`.
The :math:`3\times N` projection matrix :math:`[G_{s}]` is then defined
as

.. math::

   \label{eq:MS:4}
       [G_{s}] = \begin{bmatrix}
           \cdots
   	{\hat{\mathbf g}}^{\mathcal{B}}{\hat{\mathbf g}}
   _{s_{i}} \cdots
       \end{bmatrix}

The RW inertial angular momentum vector :math:`\mathbf h_{s}` is defined as

.. math::

   \label{eq:MS:5}
       h_{s_{i}} = J_{s_{i}} (\omega_{s_{i}} + \Omega_{i})

Here :math:`\Omega_{i}` is the :math:`i^{\text{th}}` RW spin relative to
the spacecraft, and the body angular velocity is written in terms of
body and RW frame components as

.. math::

   \label{eq:MS:6}
       \mathbf\omega = \omega_{1} \hat{\mathbf b}_{1} + \omega_{2} \hat{\mathbf b}_{2} + \omega_{3} \hat{\mathbf b}_{3}
       = \omega_{s_{i}} \hat{\mathbf g}_{s_{i}} +  \omega_{t_{i}} \hat{\mathbf g}_{t_{i}} +  \omega_{g_{i}} \hat{\mathbf g}_{g_{i}}

Angular Velocity Servo Sub-System
=================================

To implement the kinematic steering control, a servo sub-system must be
included which will produce the required torques to make the actual body
rates track the desired body rates. The angular velocity tracking error
vector is defined as

.. math::

   \label{eq:MS:32}
       \delta \mathbf \omega = \mathbf\omega_{\mathcal{B}/\mathcal{B}^{\ast}} = \mathbf\omega_{\mathcal{B}/\mathcal{N}} - \mathbf\omega_{\mathcal{B}^{\ast}/\mathcal{N}}

where the :math:`\mathcal{B}^{\ast}` frame is the desired body frame
from the kinematic steering law. Note that

.. math:: \mathbf\omega_{\mathcal{B}^{\ast}/\mathcal{N}} =  \mathbf\omega_{\mathcal{B}^{\ast}/\mathcal{R}} +  \mathbf\omega_{\mathcal{R}/\mathcal{N}}

where :math:`\mathbf\omega_{\mathcal{R}/\mathcal{N}}` is obtained from the
attitude navigation solution, and
:math:`\mathbf\omega_{\mathcal{B}^{\ast}/\mathcal{R}}` is the kinematic
steering rate command. To create a rate-servo system that is robust to
unmodeld torque biases, the state :math:`\mathbf z` is defined as:

.. math::

   \label{eq:MS:34}
       \mathbf z = \int_{t_{0}}^{t_{f}}
   	{\delta\mathbf\omega}^{\mathcal{B}}{ \delta\mathbf\omega}
   \ \textrm{d}t

The rate servo Lyapunov function is defined as

.. math::

   \label{eq:MS:35}
       V_{\mathbf\omega}(\delta\mathbf\omega, \mathbf z) = \frac{1}{2} \delta\mathbf\omega ^{T} [I_{\text{RW}}] \delta\mathbf\omega + \frac{1}{2} \mathbf z ^{T} [K_{I}] \mathbf z

where the vector :math:`\delta\mathbf\omega` and tensor
:math:`[I_{\text{RW}}]` are assumed to be given in body frame
components, :math:`[K_{i}]` is a symmetric positive definite matrix. The
time derivative of this Lyapunov function is

.. math::

   \label{eq:MS:36}
       \dot V_{\mathbf\omega} = \delta\mathbf\omega^{T} \left(
           [I_{\text{RW}}] \delta\mathbf\omega' + [K_{I}] \mathbf z
       \right)

Using the identities
:math:`{\mathbf\omega}_{\mathcal{B}/\mathcal{N}}' = \dot{\mathbf\omega}_{\mathcal{B}/\mathcal{N}}`
and
:math:`\mathbf\omega_{\mathcal{R}/\mathcal{N}}' =  \dot{\mathbf\omega}_{\mathcal{R}/\mathcal{N}} -  {\mathbf\omega}_{\mathcal{B}/\mathcal{N}} \times  \mathbf\omega_{\mathcal{R}/\mathcal{N}}`,:raw-latex:`\cite{schaub}`
the body frame derivative of :math:`\delta \mathbf\omega` is

.. math::

   \label{eq:MS:37}
       \delta\mathbf \omega '= \dot{\mathbf\omega}_{\mathcal{B}/\mathcal{N}} - \mathbf\omega_{\mathcal{B}^{\ast}/\mathcal{R}} ' -  \dot{\mathbf\omega}_{\mathcal{R}/\mathcal{N}} +  {\mathbf\omega}_{\mathcal{B}/\mathcal{N}} \times  \mathbf\omega_{\mathcal{R}/\mathcal{N}}

Substituting Eqs. `[eq:MS:2] <#eq:MS:2>`__ and
`[eq:MS:37] <#eq:MS:37>`__ into the :math:`\dot V_{\mathbf\omega}`
expression in Eq. `[eq:MS:36] <#eq:MS:36>`__ yields

.. math::

   \begin{gathered}
       \label{eq:MS:38}
       \dot V_{\mathbf\omega} = \delta\mathbf\omega^{T} \Big(
           - [\tilde{\mathbf \omega}_{\mathcal{B}/\mathcal{N}}] \left(
       [I_{RW}] \mathbf\omega_{\mathcal{B}/\mathcal{N}} + [G_{s}] \mathbf h_{s}
       \right) - [G_{s}] \mathbf u_{s} + \mathbf L + [K_{I}] \mathbf z
       \\
       - [I_{\text{RW}}](\mathbf\omega_{\mathcal{B}^{\ast}/\mathcal{R}} ' +  \dot{\mathbf\omega}_{\mathcal{R}/\mathcal{N}} - {\mathbf\omega}_{\mathcal{B}/\mathcal{N}} \times  \mathbf\omega_{\mathcal{R}/\mathcal{N}})
       \Big)
   \end{gathered}

Let :math:`[P]^{T} = [P]>` be a symmetric positive definite rate
feedback gain matrix. The servo rate feedback control is defined as

.. math::

   \begin{gathered}
       \label{eq:MS:39}
       [G_{s}]\mathbf u_{s} = [P]\delta\mathbf\omega + [K_{I}]\mathbf z - [\tilde{\mathbf\omega}_{\mathcal{B}^{\ast}/\mathcal{N}}]
       \left( [I_{\text{RW}}] \mathbf\omega_{\mathcal{B}/\mathcal{N}} + [G_{s}] \mathbf h_{s} \right)
       \\
       - [I_{\text{RW}}](\mathbf\omega_{\mathcal{B}^{\ast}/\mathcal{R}} ' +  \dot{\mathbf\omega}_{\mathcal{R}/\mathcal{N}} -  {\mathbf\omega}_{\mathcal{B}/\mathcal{N}} \times  \mathbf\omega_{\mathcal{R}/\mathcal{N}}) + \mathbf L
   \end{gathered}

Defining the right-hand-side as :math:`\mathbf L_{r}`, this is rewritten in
compact form as

.. math:: [G_{s}]\mathbf u_{s} = -\mathbf L_{r}

The array of RW motor torques can be solved with the typical minimum
norm inverse

.. math:: \mathbf u_{s} = [G_{s}]^{T}\left( [G_{s}][G_{s}]^{T}\right)^{-1} (- \mathbf L_{r})

To analyze the stability of this rate servo control, the
:math:`[G_{s}]\mathbf u_{s}` expression in Eq. `[eq:MS:39] <#eq:MS:39>`__ is
substituted into the Lyapunov rate expression in
Eq. `[eq:MS:38] <#eq:MS:38>`__.

.. math::

   \begin{aligned}
       \label{eq:MS:42}
       \dot V_{\omega} &= \delta\mathbf\omega^{T} \Big(
           - [P]\delta\mathbf\omega - [\tilde{\mathbf \omega}_{\mathcal{B}/\mathcal{N}}] \left(
       [I_{RW}] \mathbf\omega_{\mathcal{B}/\mathcal{N}} + [G_{s}] \mathbf h_{s}
       \right)
       + [\tilde{\mathbf\omega}_{\mathcal{B}^{\ast}/\mathcal{N}}]
       \left( [I_{\text{RW}}] \mathbf\omega_{\mathcal{B}/\mathcal{N}} + [G_{s}] \mathbf h_{s} \right)
       \Big )
       \nonumber \\
       &= \delta\mathbf\omega^{T} \Big( - [P]\delta\mathbf\omega
       - [\widetilde{\delta\mathbf \omega}] \left(
       [I_{RW}] \mathbf\omega_{\mathcal{B}/\mathcal{N}} + [G_{s}] \mathbf h_{s}
       \right)
       \Big )
       \nonumber \\
       &= - \delta\mathbf\omega ^{T} [P] \delta\mathbf\omega < 0
   \end{aligned}

Thus, in the absence of unmodeled torques, the servo control in
Eq. `[eq:MS:39] <#eq:MS:39>`__ is asymptotically stabilizing in rate
tracking error :math:`\delta\mathbf\omega`.

Next, the servo robustness to unmodeled external torques is
investigated. Let us assume that the external torque vector
:math:`\mathbf L` in Eq. `[eq:MS:2] <#eq:MS:2>`__ only approximates the true
external torque, and the unmodeled component is given by
:math:`\Delta \mathbf L`. Substituting the true equations of motion and the
same servo control in Eq. `[eq:MS:39] <#eq:MS:39>`__ into the Lyapunov
rate expression in Eq. `[eq:MS:36] <#eq:MS:36>`__ leads to

.. math::

   \label{eq:MS:43}
       \dot V_{\omega} = - \delta\mathbf\omega ^{T} [P] \delta\mathbf\omega - \delta\mathbf\omega ^{T} \Delta \mathbf L

This :math:`\dot V_{\omega}` is no longer negative definite due to the
underdetermined sign of the :math:`\delta\mathbf\omega ^{T} \Delta \mathbf L`
components. Equating the Lyapunov rates in
Eqs. `[eq:MS:36] <#eq:MS:36>`__ and `[eq:MS:43] <#eq:MS:43>`__ yields
the following servo closed loop dynamics:

.. math::

   \label{eq:MS:44}
       [I_{\text{RW}}]\delta\mathbf\omega' + [P]\delta\mathbf\omega + [K_{I}]\mathbf z = \Delta\mathbf L

Assuming that :math:`\Delta\mathbf L` is either constant as seen by the body
frame, or at least varies slowly, then taking a body-frame time
derivative of Eq. `[eq:MS:44] <#eq:MS:44>`__ is

.. math::

   \label{eq:MS:45}
       [I_{\text{RW}}]\delta\mathbf\omega'' + [P]\delta\mathbf\omega' + [K_{I}]\delta \mathbf \omega = \Delta\mathbf L' \approx 0

As :math:`[I_{\text{RW}}]`, :math:`[P]` and :math:`[K_{I}]` are all
symmetric positive definite matrices, these linear differential
equations are stable, and :math:`\delta\mathbf\omega\rightarrow0` given that
assumption that :math:`\Delta\mathbf L' \approx 0`.

Unit Test
=========

The unit test for this module ``test_MRP_feedback`` tests a set of gains
:math:`K,K_i,P` on a rigid body with no external torques, and with a
fixed input reference attitude message. The torque requested by the
controller is evaluated against python computed torques at 0s, 0.5s, 1s,
1.5s and 2s to within a tolerance of :math:`10^{-8}`. After 1s the
simulation is stopped and the reset() function is called to check that
integral feedback related variables are properly reset. The following
permutations are run:

- The test is run for a case with error integration feedback
  (:math:`k_i=0.01`) and one case where :math:`k_i` is set to a negative
  value, resulting in a case with no integrator.

- The RW array number is configured either to 4 or 0

- The integral limit term is set to either 0 or 20

- The RW availability message is tested in 3 manners. Either the
  availability message is not written where all wheels should default to
  being available. If the availability message is written, then the RWs
  are either zero to available or not available.

- The values of :math:`^B\omega'_{B^*R}` and\ :math:`^B\omega_{B^*R}`
  are varied between a non-zero 3x1 vector to a zero vector

All permutations of these test cases are expected to pass.

User Guide
==========

This module requires the following variables:

- :math:`^B\mathbf{\omega}_{BR}` as ``guidCmdData.omega_BR_B``

- :math:`^B\mathbf{\omega}_{RN}` as ``guidCmdData.omega_RN_B``

- :math:`^B\dot{\mathbf{\omega}}_{RN}` as ``guidCmdData.domega_RN_B``

- :math:`[I]`, the inertia matrix of the body as
  ``vehicleConfigOut.ISCPntB_B``

- :math:`\Omega_i`, speed of each reaction wheel in
  ``rwSpeedMessage.wheelSpeeds``

- Gains :math:`k,P` in ``moduleConfig``.

- The integral gain :math:`K_i` in ``moduleConfig``. Setting this
  variable to a negative number disables the error integration for the
  controller, leaving just PI terms.

- The value of ``integralLimit``, used to limit the degree of integrator
  windup and reduce the chance of controller saturation. The integrator
  is required to maintain asymptotic tracking in the presence of an
  external disturbing torque. For this module, the integration limit is
  applied to each element of the integrated error vector :math:`z`, and
  any elements greater than the limit are set to the limit instead.

- Commanded body rates omegap_BastR_B and omega_BastR_B

Module Setup
------------

The module is configured by::

    module = rateServoFullNonlinear.RateServoFullNonlinear()
    module.modelTag = "rateServoFullNonlinear"
    module.setKi(0.01)
    module.setP(150.0)
    module.setIntegralLimit(20)
    module.setKnownTorquePntB_B([1., 1., 1.])

Finally, the module is added to the simulation using::

    unitTestSim.AddModelToTask(unitTaskName, module)
