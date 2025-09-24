Executive Summary
-----------------
This attitude guidance module create a reference attitude message that points in fixed inertial direction.

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
    * - attRefOutMsg
      - :ref:`AttRefMsgPayload`
      - attitude reference output message


Reference Frame Generation
--------------------------
The modules requires the desired reference orientation in terms of the MRP set :math:`\mathbf{\sigma}_{R_0 N}`.
This input is only set once and does not have to be changed. Let us designate :math:`\mathcal{R}` as the output
generated reference frame. Since the fixed-pointing is inertial:

.. math::
    \mathbf{\sigma}_{RN} = \mathbf{\sigma}_{R_0 N} \\
    \mathbf{\omega}_{RN} = \dot{\mathbf{\omega}}_{RN} = 0

User Guide
----------
The required module configuration is::

    attGuid = inertial3D.inertial3D()
    attGuid.setSigmaR0N(sigma_R0N)
