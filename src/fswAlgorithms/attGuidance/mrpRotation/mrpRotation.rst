Executive Summary
=================

This module creates a dynamic reference frame attitude state message where the initial orientation relative to the
input reference frame is specified through an MRP set, and the angular velocity vector is held fixed as seen by the
resulting reference frame.

Message Connection Descriptions
===============================

The following table lists all the module input and output messages.  The module msg connection is set by the
user from python.  The msg type contains a link to the message structure definition, while the description
provides information on what this message is used for.

.. _ModuleIO_mrpRotation:
.. figure:: _Documentation/Figures/moduleIO.pdf
    :align: center
    :name: fig:moduleIO

    Figure 1: ``mrpRotation()`` Module I/O Illustration


.. list-table:: Module I/O Messages
    :widths: 25 25 50
    :header-rows: 1

    * - Msg Variable Name
      - Msg Type
      - Description
    * - attRefOutMsg
      - :ref:`AttRefMsgPayload`
      - name of the output message containing the Reference
    * - attRefInMsg
      - :ref:`AttRefMsgPayload`
      - name of the guidance reference input message
    * - desiredAttInMsg
      - :ref:`AttStateMsgPayload`
      - (optional) name of the incoming message containing the desired MRP set

Model Description
=================

The purpose of this ``mrpRotation`` module is to add a constant rotation
relative to the input frame :math:`\mathcal{R}_{0}`. The output
reference frame is called :math:`\mathcal{R}`. The initial orientation
is specified through an MRP set
:math:`\mathbf\sigma_{R/R0}`, while the :math:`\mathcal{R}`-frame angular
velocity vector :math:`{}^{\mathcal{R}}{\mathbf\omega}_{R/R0}`
is held constant in this module.

Assume that the input reference frame :math:`\mathcal{R}_{0}` is given
through an attitude state input message containing
:math:`\mathbf\sigma_{R_{0}/N}`,
:math:`{}^{\mathcal{N}}{\mathbf\omega}_{R_{0}/N}` and
:math:`{}^{\mathcal{N}}{\dot{\mathbf\omega}}_{R_{0}/N}`
as illustrated in :ref:`fig:moduleIO`.
The MRP set is mapped into the corresponding Direction Cosine Matrix (DCM) using

.. math:: [R_{0}N] = [R_{0}N ( \mathbf\sigma_{R_{0}/N})]

The goal of the motion is to compute the attitude of :math:`\mathcal{R}`
relative to input frame :math:`\mathcal{R}_{0}` such that

.. math::
   \begin{align}
   	\dot{\mathbf\sigma}_{R/R_{0}} &= \frac{1}{4} [B(\mathbf\sigma_{R/R_{0}})] {}^{\mathcal{R}}{\mathbf\omega}_{R/R_{0}}
    & \label{eq:mRot1} \\
   	\frac{{}^{\mathcal{R}}{\textrm{d}} {\mathbf\omega}_{R/R_{0}}}{\textrm{d}t} &= \mathbf 0 & \label{eq:mRot2}
   \end{align}

Assume the initial :math:`\mathbf\sigma_{R/R_{0}}(t_{0})` set and the
:math:`\mathcal{R}`-frame relative invariant
:math:`{}^{\mathcal{R}}{\mathbf\omega}_{R/R_{0}}` vector are provided to the module. The current
:math:`\mathbf\sigma_{R/R_{0}}(t_{0})` value is then obtained by
Eq.:ref:`eq:mRot1`. The current DCM of the
:math:`\mathcal{R}`-frame is thus

.. math::

   \label{eq:mRot3}
   	[RN] = [RR_{0}(\mathbf\sigma_{R/R_{0}}(t) ] [R_{0}N]

Next, the angular velocity vector is transformed to inertial frame
:math:`\mathcal{N}`-frame components using

.. math::

   \label{eq:mRot4}

   	{}^{\mathcal{N}}{\mathbf\omega}_{R/R_{0}} = [RN]^{T}{}^{\mathcal{R}}{\mathbf\omega}_{R/R_{0}}

to find the inertial angular velocity of the output reference frame:

.. math::

   \label{eq:mRot5}

   	{}^{\mathcal{N}}{\mathbf\omega}_{R/N} = {}^{\mathcal{N}}{\mathbf\omega}_{R/R_{0}} +
    {}^{\mathcal{N}}{\mathbf\omega}_{R_{0}/N}

Finally, the inertial angular acceleration of the output reference frame
is found using the transport theorem:

.. math::

   \label{eq:mRot6}
   	\dot{\mathbf\omega}_{R/N} =
   	\frac{
   	{}^{\mathcal{R}}{\textrm{d}}
    {\mathbf\omega}_{R/R_{0}}}{\textrm{d}t}
   	 + \mathbf\omega_{R/N} \times {\mathbf\omega}_{R/R_{0}}
   	+ \dot{\mathbf\omega}_{R_{0}/N}
   	= \mathbf\omega_{R_{0}/N} \times {\mathbf\omega}_{R/R_{0}}
   	+ \dot{\mathbf\omega}_{R_{0}/N}

where
:math:`\mathbf\omega_{R/N} \times {\mathbf\omega}_{R/R_{0}} = (\mathbf\omega_{R/R_{0}} +\mathbf\omega_{R_{0}/N})
\times {\mathbf\omega}_{R/R_{0}} = \mathbf\omega_{R_{0}/N} \times {\mathbf\omega}_{R/R_{0}}`
is used. Expressed in :math:`\mathcal{N}` frame components, this vector
equation is numerically evaluated using:

.. math::

   \label{eq:mRot7}

   	{}^{\mathcal{N}}{\dot{\mathbf\omega}}
   _{R/N} =
   	{}^{\mathcal{N}}{\mathbf\omega}
   _{R_{0}/N} \times
   	{}^{\mathcal{N}}{\mathbf\omega}
   _{R/R_{0}}
   +
   	{}^{\mathcal{N}}{\dot{\mathbf\omega}}
   _{R_{0}/N}

Module Functions
================

The ``mrpRotation`` module has the following design goals

- **Constant Spin**: The angular velocity vector between the input and
  output frame is constant as seen by output reference frame

- **MRP attitude representation**: The initial and output attitude is
  described through an MRP coordinate set

- **Flexible Setup**: The desired rotation state can be described
  through an initial MRP and angular velocity vector specified in module
  internal variables, or read in through a Xmera :ref:`AttStateMsgPayload`
  message.

Module Assumptions and Limitations
==================================

- On reset the next time step doesn’t yield an integration as the
  integration time evaluation requires at least a second time step.

- If the desired rotation states are read in with an input message, then
  this message is checked each update cycle for new content. On reset
  the commanded frame states are reset to zero such that they are
  re-read in again in the next update cycle.

- If the desired rotation is specified with module internal states, then
  on reset the prior internal states are re-used unless they are
  over-written after the reset call.

User Guide
==========

Specifying Desired Rotation
---------------------------

If the ``mrpRotation`` module is set directly with the desired rotation
states, then the modules variables ``sigma_RR0`` and ``omega_RR0_R`` must
be set.

If instead the desired rotation states are to be read in, then the
input message name ``desiredAttInMsg`` must be specified, and a
corresponding message of type :ref:`AttStateMsgPayload` created.

Required Input and Output Messages
----------------------------------

The :math:`\mathcal{R}_{0}` input reference frame state message is
specified through ``attRefInMsg``. The output message name is specified
through the ``attRefOutMsg``.

Module Reset Behavior
---------------------

If the module is reset, then the ``priorTime`` flag is reset, meaning it
take another time step to compute the sampling period used to integrate
the kinematic differential equations.

Module Setup
------------

The module is configured by::

    module = mrpRotation.mrpRotation()
    module.modelTag = "mrpRotation"

The desired rotation state can be directly specified by::

    sigma_RR0 = np.array([0.3, .5, 0.0])
    module.setSigmaRR0(sigma_RR0)
    omega_RR0_R = np.array([0.1, 0.0, 0.0]) * mc.D2R
    module.setOmegaRR0(omega_RR0_R)

Alternatively, the desired rotation state can be specified with the input message ``desiredAttInMsg``::

    module.desiredAttInMsg.subscribeTo(desiredAttMsg)

Finally, the module is added to the simulation using::

    unitTestSim.AddModelToTask(unitTaskName, module)
