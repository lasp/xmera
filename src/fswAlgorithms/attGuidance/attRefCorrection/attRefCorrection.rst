Executive Summary
-----------------
This module reads in the attitude reference message and adjusts it by a fixed rotation.  This allows a general
body-fixed frame
:math:`B` to align with this corrected reference frame :math:`R`.

Message Connection Descriptions
-------------------------------
The following table lists all the module input and output messages.
The module msg connection is set by the user from python.
The msg type contains a link to the message structure definition, while the description
provides information on what this message is used for.

.. list-table:: Module I/O Messages
    :widths: 25 25 50
    :header-rows: 1

    * - Msg Variable Name
      - Msg Type
      - Description
    * - attRefInMsg
      - :ref:`AttRefMsgPayload`
      - attitude reference input message
    * - attRefOutMsg
      - :ref:`AttRefMsgPayload`
      - corrected attitude reference input message

Detailed Module Description
---------------------------

This module is an attitude reference message feed-through module where a fixed orientation offset can be applied
to the output attitude :math:`\sigma_{R/N}`.  In not all cases do we wish to drive a body-fixed
frame :math:`\mathcal B` to a reference frame :math:`\mathcal R`.  Therefore, we introduce a fixed offset :math:`\sigma_{R/R0}`
and add this to the input :math:`\sigma_{R0/N}`. This resulted in a corrected reference frame to control the
:math:`\mathcal B` to.

.. math::

    [RN] = [R R0][R0 N]

This relationship can be found by using the addition of two mrps using the rigid body kinematics library, which is what
the algorithm uses internally.

User Guide
----------

The only variable that is set with this module is the ``sigma_RR0`` MRP to rotate from the original
reference frame to corrected reference frame.
