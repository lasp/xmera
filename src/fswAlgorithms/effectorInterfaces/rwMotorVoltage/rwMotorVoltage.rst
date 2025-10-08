Executive Summary
-----------------

This module computes the RW motor voltage from the commanded torque.

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
    * - voltageOutMsg
      - :ref:`RwMotorVoltageMsgPayload`
      - RW motor voltage output message
    * - torqueInMsg
      - :ref:`RwMotorTorqueMsgPayload`
      - commanded RW motor torque input message
    * - rwParamsInMsg
      - :ref:`RWArrayConfigMsgPayload`
      - RW array configuration input message
    * - rwAvailInMsg
      - :ref:`RWAvailabilityMsgPayload`
      - (optional) RW device availability message
    * - rwSpeedInMsg
      - :ref:`RWSpeedMsgPayload`
      - (optional) RW device speed message

Model Description
=================

There two types of RW control torque interfaces, analog and digital.
This modules assumes the RW is controlled through a set of voltages sent
to the RW motors. This module is developed in a general manner where a
voltage deadband is assumed and the module can be run in a pure
open-loop manner, or with a closed-loop torque tracking control mode.
Finally, if a RW availability message is present, then the RW is set to
zero if the corresponding availability is set to ``UNAVAILABLE``.

.. figure:: Figures/us2V.pdf
   name: fig:us2V

   Illustration of RW motor torque to voltage conversion

Open-loop voltage conversion
----------------------------

This module requires the RW configuration message to contain the maximum
RW motor torque values :math:`u_{\text{max}}`. The user must specify the
minimum and maximum output voltages as shown in
Figure `1 <#fig:us2V>`__. The minimum voltage is a voltage below which
the motor doesn’t apply a torque, i.e. a deadzone.

Let the intermediate voltage value :math:`V_{\text{int}}` as

.. math::

   \label{eq:rwMV:1}
   V_{\text{int}} = \frac{V_{\text{max}} - V_{\text{min}}}{u_{\text{max}}} u_{s}

The output voltage is thus determined through

.. math:: V = V_{\text{int}} + V_{\text{min}} *\text{sgn}(V_{\text{int}} )

RW Availability
---------------

If the input message name ``rwAvailInMsg`` is defined, then the RW
availability message is read in. The voltage mapping is only performed
if the individual RW availability setting is ``AVAILABLE``. If it is
``UNAVAILABLE`` then the output voltage is set to zero.

Closed-loop commanded torque tracking
-------------------------------------

The requested RW motor torque is given by :math:`u_{s}`. The RW wheel
speed :math:`\Omega` is monitored to see if the actual torque being
applied matches the commanded torque. Let :math:`J_{s}` be the RW spin
inertia about the RW spin axis :math:`\hat{\bm g}_{s}`. In the following
development the motor torque equation is approximated as

.. math:: u_{s} = J_{s} \dot\Omega

where the assumption is made that the spacecraft angular accelerations
are small compared to the RW angular accelerations. The
:math:`\dot\Omega` term is digitally evaluated using a backwards
difference method:

.. math:: \dot\Omega_{n} = \frac{\Omega_{n} - \Omega_{n-1}}{\Delta t}

Care is taken that the old RW speed information :math:`\Omega_{n-1}` is
not used unless a history of wheel speeds is available, in particular,
after a module reset. Thus, the actual RW torque is evaluated as

.. math:: u_{n} = J_{s} \dot\Omega_{n}

Finally, the closed loop motor torque value is computed with a
proportional feedback component as

.. math:: u_{s,CL} = u_{s} - K (u_{n} - u_{s})

where :math:`K>0` is a positive feedback gain value. Finally, this
:math:`u_{s,CL}` is fed to the voltage conversion process in
Eq. `[eq:rwMV:1] <#eq:rwMV:1>`__.

Saturation and Dead Band
------------------------

If the calculated voltage is outside of :math:`\pm V_{\mathrm{max}}`,
then the voltage is saturated at the :math:`\pm V_{\mathrm{max}}` value.
Note, this corresponds to the reaction wheel torques being saturated.
Similarly, if the calculated voltage is inside
:math:`\pm V_{\mathrm{min}}`, then the voltage is set to
:math:`\pm V_{\mathrm{min}}`. This simulates the dead band. If the
:math:`V_{\mathrm{min}} = 0`, then there is no dead band.

Model Assumptions and Limitations
=================================

This code makes the following assumptions:

- **Linear**: This code assumes that there is a linear relationship
  between the torque desired and the voltage required to create that
  torque.

User Guide
==========

The module is configured by::

    module = rwMotorVoltage.RwMotorVoltage(minVoltageMagnitude, maxVoltageMagnitude)
    module.modelTag = "rwMotorVoltage"
    module.setVoltageRange(newMinVoltageMagnitude, newMaxVoltageMagnitude)  # change voltage range if desired
    module.setGainK(gain)
