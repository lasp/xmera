Executive Summary
-----------------
This module implements a feedback control law to damp the angular rates of the spacecraft until they are brought to
zero.

Message Connection Descriptions
-------------------------------
The following table lists all the module input and output messages. The msg type contains a link to the message
structure definition, while the description provides information on what this message is used for.

.. list-table:: Module I/O Messages
    :widths: 25 25 50
    :header-rows: 1

    * - Msg Variable Name
      - Msg Type
      - Description
    * - vehConfigInMsg
      - :ref:`VehicleConfigMsgPayload`
      - Attitude guidance input message
    * - guidInMsg
      - :ref:`AttGuidMsgPayload`
      - Vehicle configuration input message
    * - cmdTorqueOutMsg
      - :ref:`CmdTorqueBodyMsgPayload`
      - Commanded torque output message

Detailed Module Description
---------------------------
The control is similar to :ref:`mrpPD`, but does not feed back on the orientation error:

.. math::
    {\bf L}_{r} =  -K \pmb\sigma - [P] \delta\pmb\omega   + [I](\dot{\pmb\omega}_{r} - [\tilde{\pmb\omega}]\pmb\omega_{r})
    			+[\tilde{\pmb \omega}_{r}] ]
    			[I]\pmb\omega   - \bf L

where :math:`P` is a positive, user-defined scalar quantity.

User Guide
----------
The required module configuration is::

    module = rateDamp.RateDamp()
    module.modelTag = "rateDamp"
    module.setDerivativeGainP(P)
    module.setKnownTorquePntB_B(knownTorquePntB_B)
