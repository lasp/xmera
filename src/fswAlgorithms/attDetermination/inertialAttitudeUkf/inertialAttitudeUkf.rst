Executive Summary
-----------------

This module filters incoming attitude and rate measurements along with reaction wheel data in order to get the best
possible inertial attitude estimate. The filter used is an unscented Kalman filter using the Modified Rodrigues
Parameters (MRPs) as a non-singular attitude measure. Attitude measurements (e.g. from star trackers) can be
added from multiple sources via ``addAttitudeInput``, and rate measurements are read through a dedicated input
message. The rate measurements are packed in an :ref:`STAttMsgPayload` but may originate from either star tracker
measurements or gyroscope measurements.

More information on can be found in the
:download:`PDF Description </../../src/fswAlgorithms/attDetermination/InertialUKF/_Documentation/Xmera-inertialUKF-20190402.pdf>`

Message Connection Descriptions
-------------------------------
The following table lists all the module input and output messages.  The module msg connection is set by the
user from python.  The msg type contains a link to the message structure definition, while the description
provides information on what this message is used for.

.. list-table:: Module I/O Messages
    :widths: 15 15 70
    :header-rows: 1

    * - Msg Variable Name
      - Msg Type
      - Description
    * - navAttitudeOutputMsg
      - :ref:`NavAttMsgPayload`
      - navigation output message
    * - inertialFilterOutputMsg
      - :ref:`FilterMsgPayload`
      - output filter data message
    * - attitudeResidualMsg
      - :ref:`FilterResidualsMsgPayload`
      - output residual data message for attitude measurements
    * - rateResidualMsg
      - :ref:`FilterResidualsMsgPayload`
      - output residual data message for rate measurements
    * - vehicleConfigMsg
      - :ref:`VehicleConfigMsgPayload`
      - spacecraft vehicle configuration input message
    * - rwArrayConfigMsg
      - :ref:`RWArrayConfigMsgPayload`
      - reaction wheel parameter input message.  Can be an empty message if no RW are included.
    * - rwSpeedMsg
      - :ref:`RWSpeedMsgPayload`
      - reaction wheel speed input message.  Can be an empty message if no RW are included.
    * - rateDataInMsg
      - :ref:`STAttMsgPayload`
      - rate measurement input message packed in an :ref:`STAttMsgPayload`; rates may come from either star tracker measurements or a gyroscope
