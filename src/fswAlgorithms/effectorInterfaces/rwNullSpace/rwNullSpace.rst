Executive Summary
-----------------

This module uses the Reaction Wheel (RW) null space to slow down the wheels. The resulting motor torques are super
imposed on top of the attitude feedback control RW motor torques.


Message Connection Descriptions
-------------------------------
The following table lists all the module input and output messages.  The module msg connection is set by the
user from python.  The msg type contains a link to the message structure definition, while the description
provides information on what this message is used for.

.. _ModuleIO_NullSpace:
.. figure:: /../../src/fswAlgorithms/effectorInterfaces/rwNullSpace/_Documentation/Images/moduleImgNullSpace.svg
    :align: center

    Figure 1: ``rwNullSpace()`` Module I/O Illustration

.. list-table:: Module I/O Messages
    :widths: 25 25 50
    :header-rows: 1

    * - Msg Variable Name
      - Msg Type
      - Description
    * - rwMotorTorqueInMsg
      - :ref:`RwMotorTorqueMsgPayload`
      - RW motor torque input message
    * - rwSpeedsInMsg
      - :ref:`RWSpeedMsgPayload`
      - RW speed message
    * - rwDesiredSpeedsInMsg
      - :ref:`RWSpeedMsgPayload`
      - (optional) input message with the desired RW speeds
    * - rwConfigInMsg
      - :ref:`RWConstellationMsgPayload`
      - RW constellation configuration input message
    * - rwMotorTorqueOutMsg
      - :ref:`RwMotorTorqueMsgPayload`
      - RW motor torque output message


Module Description
==================

Module Purpose
--------------

This module aims to drive the reaction wheel (RW) speeds to desired
values by using the null space of the RW array. There are three required
input messages:

#. The feedback control torque which is mapped onto the RW motor torque
   solution space. The message type is ``RWArrayTorqueIntMsg``, the same
   message type as the module output message.

#. The RW speed array message of type ``RWSpeedIntMsg``.

#. The RW configuration message contains the RW spin axes information.
   The message type is ``RWConstellationFswMsg``.

There is one optional input message which provides the desired RW
speeds. If this message is not connected, then the desired speeds
default to zero values.

The module output message is a RW motor torque array message that sums
the input control torque solution with the new null space torque
solution that will reduce the RW speeds.

RW Null Space Mathematics
-------------------------

Let :math:`N` be the number of RWs present, while
:math:`\hat{\bm g}_{s_{i}}` is the :math:`i^{\text{th}}` RW spin axis
unit direction vector. The RW spin axes matrix is defined as

.. math:: [G_{s}] = [\hat{\bm g}_{s_{1}} \cdots \hat{\bm g}_{s_{N}}]

The null motion RW projection matrix :math:`[\tau]` is given
by:raw-latex:`\cite{schaub}`

.. math:: [\tau] = [I_{N\times N}] - [G_{s}]^{T} \left( [G_{s}] [G_{s}]^{T} \right)^{-1} [G_{s}]

This project matrix maps any vector :math:`\bm d` into the null-space of
:math:`[G_{s}]` such that no torque is exerted onto the spacecraft. As a
result, these null-motion solution never impact the stability or
performance of the RW attitude control solution. This concept is
illustrated through:

.. math::

   \begin{aligned}
   	[G_{s}] [\tau] &= [G_{s}] \left( [I_{N\times N}] - [G_{s}]^{T} \left( [G_{s}] [G_{s}]^{T} \right)^{-1} [G_{s}] \right)
   	\\
   	&= [G_{s}] - [G_{s}] [G_{s}]^{T} \left( [G_{s}] [G_{s}]^{T}] \right)^{-1} [G_{s}] \\
   	&= [G_{s}] - [G_{s}] \\
   	&= [0_{N\times N}]
   \end{aligned}

RW Wheel Speed Reduction Control
--------------------------------

Let :math:`J_{s_{i}}>0` be the RW inertia about the spin axis
:math:`\hat{\bm g}_{s_{i}}`, :math:`\Omega_{i}` be the RW spin speed,
and :math:`\dot{\bm \omega}` be the inertial spacecraft angular
acceleration vector. Let :math:`\Omega_{i,d}` be the desired wheel
speeds, and

.. math:: \Delta\Omega_i = \Omega_i - \Omega_{i,d}

be the wheel speed tracking errors.

The RW motor torque equation is given by:raw-latex:`\cite{schaub}`

.. math:: u_{s_{i}} = J_{s_{i}} (\dot\Omega_{i} + \hat{\bm g}_{s_{i}}^{T} \dot{\bm \omega} )

Assuming that the spacecraft angular accelerations are much smaller than
the wheel accelerations, this is approximated as

.. math:: u_{s_{i}} \approx J_{s_{i}} \dot\Omega_{i}

Let :math:`d_{i}` be the desired torque to drive the
:math:`i^{\text{th}}` RW spin rate :math:`\Omega_{i}` to the desired
rate :math:`\Omega_{i,d}` be given through

.. math:: d_{i} = - K \Delta\Omega_{i}

where the feedback gain :math:`K>0`. Then setting
:math:`u_{s_{i}} = d_{i}` ideally provides the stable closed loop
response

.. math:: J_{s_{i}} \dot\Omega_{i} + K\Delta\Omega_{i} = 0

as :math:`\dot\Omega_{i,d} = 0`.

Let :math:`\bm d` be the :math:`N\times 1` array of desired RW
decelerating motor torques given by

.. math:: \bm d = -K \Delta\bm\Omega

If this RW motor torque were directly applied then a non-zero torque
would be produced onto the spacecraft causing attitude deviations.
Instead, this desired despin torque :math:`\bm d` is mapped through
:math:`[\tau]` onto the null space of the RW array using

.. math:: \bm u_{s,\text{null}} = [\tau] \bm d

Assume the attitude feedback RW motor control solution is given by
:math:`\bm u_{s,\text{cont}}`, then final module RW motor torque array
is the sum of these two torques.

.. math:: \bm u_{s} = \bm u_{s,\text{cont}} + \bm u_{s,\text{null}}

Module Functions
================

This module has the following functions:

- **Evaluate RW null projection matrix :math:`[\tau]`**: When reset the
  module will pull in the current RW configuration data and create the
  null motion projection matrix. This matrix remains fixed unit the
  module is reset again.

- **Compute a RW deceleration torque**: With each update call the module
  computes a decelerating RW torque solution that lies in the null space
  of the RW array.

- **Output a net RW motor torque solution**: The module combined the
  feedback control torque and the null space torque to slow down the RW
  speeds and outputs a net solution solution.

Module Assumptions and Limitations
==================================

The module assumes all RW devices are operating and available. It also
assumes the RW spin axes don’t change during the regular update cycles.

User Guide
==========

The module must have the feedback gain ``OmegaGain`` defined. This must
be a positive value. Further, all 3 input message connections must be
setup.

Module Setup
============

The required module configuration is::

    module = rwNullSpace.RwNullSpace()
    module.modelTag = "rwNullSpace"
    module.setOmegaGain(gain)
