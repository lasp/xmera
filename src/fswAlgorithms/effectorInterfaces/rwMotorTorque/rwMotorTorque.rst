Executive Summary
-----------------

This module maps a desired torque to control the spacecraft, and maps it to the available wheels using a minimum norm
inverse fit.

The optional wheel availability message is used to include or exclude particular reaction wheels from the torque
solution. The desired control torque can be mapped onto particular orthogonal control axes to implement a partial
solution for the overall attitude control torque.

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
    * - rwMotorTorqueOutMsg
      - :ref:`RwMotorTorqueMsgPayload`
      - RW motor torque output message
    * - vehControlInMsg
      - :ref:`CmdTorqueBodyMsgPayload`
      - commanded vehicle control torque input message
    * - vehControlIn2Msg
      - :ref:`CmdTorqueBodyMsgPayload`
      - (optional) additional commanded vehicle control torque input message
    * - rwParamsInMsg
      - :ref:`RWArrayConfigMsgPayload`
      - RW array configuration input message
    * - rwAvailInMsg
      - :ref:`RWAvailabilityMsgPayload`
      - (optional) RW device availability message

Model Description
=================

Module Input and Output Behavior
--------------------------------

As illustrated in Figure 1, this module takes an attitude control torque
:math:`{\vphantom{\bm L_{r}}}^{\mathcal{B}\!}{\bm L_{r}}` and maps the
vector onto the specific control axes, :math:`\hat{\bm c}_{i}`. This
allows only a subset of
:math:`{\vphantom{\bm L_{r}}}^{\mathcal{B}\!}{\bm L_{r}}` to be
implemented with the Reaction Wheels (RWs). The next step is to map
reduced control torque onto the available RW spin
axes,\ :math:`\hat{\bm g}_{s_j}`. The module accounts for the
availability of the reaction wheels in the case that not all wheels are
functioning appropriately or are undergoing independent analysis.

.. container:: float
   :name: fig:moduleImg

   |image|

Assume in this documentation that the number of available RWs is
:math:`m`, while the number of desired control axes
:math:`\hat{\bm c}_{i}` is :math:`n`. The number of installed RWs is
:math:`N`.

The commanded torque message is a required input message and is read in
every time step. The RW configuration message is also required, but only
read in during reset. If the RW spin axes :math:`\hat{\bm b}_{s_{i}}`
change then reset() must be called again. The RW availability message is
optional. If the availability message is not used, then all installed
RWs are assumed to be available. The output message is always the array
of RW motor torques.

Torque Mapping
--------------

The ``rwMotorTorque`` module receives a desired attitude control torque
in the body frame :math:`{\vphantom{\bm L}}^{\mathcal{B}\!}{\bm L}
_r`. If two control torques are provided via the second optional control
input message, these two are added together to compute
:math:`{\vphantom{\bm L}}^{\mathcal{B}\!}{\bm L}
_r`. This torque is the net control torque that should be applied to the
spacecraft. Let :math:`\hat{\bm g}_{s_{i}}` be the individual RW spin
axis, while :math:`\bm u_{s}` is the :math:`m`-dimensional array of
motor torques. The :math:`3\times m` projection matrix :math:`[G_{s}]`
then maps the control torque on motor torques using

.. math::

   \label{eq:rwtm1}
   	[G_{s}] \bm u_{s} = (-
   	{\vphantom{\bm L}}^{\mathcal{B}\!}{\bm L}
   _r)

The project matrix is defined as

.. math::

   \label{eq:rwtm2}
   	[G_{s}] = \begin{bmatrix}
   		\hat{\bm g}_{s_{1}} & \cdots & \hat{\bm g}_{s_{m}}
   	\end{bmatrix}

Note that here :math:`\bm u_{s}` is the array of available motor
torques. The installed set of RWs could be larger than :math:`m`.

The projection matrix to map a vector in the body frame :math:`\cal B`
onto the set of control axes :math:`\hat{\bm c}_{i}` is given by

.. math::

   \label{eq:rwtm3}
   	[CB] = \begin{bmatrix}

   	{\vphantom{\hat{\bm c}}}^{\mathcal{B}\!}{\hat{\bm c}}
   _{1}^{T}
   		\\
   		\vdots \\

   	{\vphantom{\hat{\bm c}}}^{\mathcal{B}\!}{\hat{\bm c}}
   _{n}^{T}
   	\end{bmatrix}

where :math:`n \le 3`.

To map the requested torque onto the control axes,
Eq. `[eq:rwtm1] <#eq:rwtm1>`__ is pre-multifplied by :math:`[CB]` to
yield

.. math::

   \label{eq:rwtm4}
   	[CB][G_{s}] \bm u_{s} = [CG_{s}] \bm u_{s} = [CB](-
   	{\vphantom{\bm L}}^{\mathcal{B}\!}{\bm L}
   _r) =
   	{\vphantom{\bm L}}^{\mathcal{C}\!}{\bm L}
   _{r}

Note that :math:`[CG_{s}]` is a :math:`n\times m` matrix.

The module assumes that :math:`m\ge n` such that there are enough RWs
available to implement :math:`{\vphantom{\bm L}}^{\mathcal{C}\!}{\bm L}
_{r}`. If not, then the output motor torques are set to zero with
:math:`\bm u_{s} = \bm 0`.

To invert Eq. `[eq:rwtm4] <#eq:rwtm4>`__ a minimum norm solution is used
yielding:

.. math::

   \bm u_{s}  = [CG]^T \left([CG][CG]^T\right)^{-1}\
   	{\vphantom{\bm L}}^{\mathcal{C}\!}{\bm L}
   _{r}

The final step is to map the array of available motor torques
:math:`\bm u_{s}` onto the output array of installed RW motor torques.

RW Availability
---------------

If the input message ``rwAvailInMsg`` is linked, then the RW
availability message is read in. The torque mapping is only used for
RW’s whose availability setting is ``AVAILABLE``. If it is
``UNAVAILABLE`` then that RW output torque is set to zero.

Model Functions
===============

The code performs the following functions:

- **Maps control torque vector onto available reaction wheels**: Takes a
  desired body-frame torque from ``CmdTorqueBodyIntMsg`` and maps it
  onto the RW axes.

- **Removes torque from unavailable reaction wheels**: The module
  observes the availability of the RWs and maps the torques to only
  available reaction wheels.

Model Assumptions and Limitations
=================================

This code makes the following assumptions:

- The number of available wheels must be equal or larger than the number
  of control axes. If this is not the case a zero output motor torque
  message is produced.

.. |image| image:: Figures/moduleImg.pdf
   :width: 75.0%

Module Setup
------------

The module is configured by::

    module = rwMotorTorque.RwMotorTorque()
    module.modelTag = "rwMotorTorque"
    control_axes_B = [[1, 0, 0], [0, 1, 0], [0, 0, 0]]
    module.setControlAxes(control_axes_B)
