Executive Summary
-----------------
This module computes a reference attitude frame using the TRIAD method that simultaneously satisfies multiple pointing constraints. The TRIAD Method determines spacecraft attitude by aligning two sets of vector observations. the input arguments are two non-parallel vectors in the body frame (
:math:`{}^\mathcal{B}\hat{h}_1` and :math:`{}^\mathcal{B}\hat{a}_1`
) and two targeted non-parallel vector in the inertial frame. The mathematical details of the triad algorithm can be found in R. Calaon phd "Guidance, Control and Momentum Management of
Spacecraft with Multiple Pointing Constraints ". it creates a rotational matrix that prioritise the alignment of the body-frame direction :math:`{}^\mathcal{B}\hat{h}_1` with a the targeted reference direction :math:`{}^\mathcal{N}\hat{h}_\text{ref}`, while taking is account all other constrains of the state.

Message Connection Descriptions
-------------------------------
The following table lists all the module input and output messages. The msg type contains a link to the message structure definition, while the description
provides information on what this message is used for.

.. figure:: /../../src/fswAlgorithms/attGuidance/triad/triad.png
    :align: center

    Figure 1: ``TRIAD``  Illustration

.. list-table:: Module I/O Messages
    :widths: 25 25 50
    :header-rows: 1

    * - Msg Variable Name
      - Msg Type
      - Description
    * - attNavInMsg
      - :ref:`NavAttMsgPayload`
      - Input message containing current attitude and Sun direction in body-frame coordinates. Note that, for the Sun direction to appear in the message, the :ref:`SpicePlanetStateMsgPayload` must be provided as input msg to :ref:`simpleNav`, otherwise the Sun direction is zeroed by default.
    * - bodyHeadingInMsg
      - :ref:`BodyHeadingMsgPayload`
      - (optional) Input message containing the body-frame direction :math:`{}^\mathcal{B}\hat{h}`. Alternatively, the direction can be specified as input parameter ``h1Hat_B``. When this input msg is connected, the input parameter is neglected in favor of the input msg.
    * - inertialHeadingInMsg
      - :ref:`InertialHeadingMsgPayload`
      - (optional) Input message containing the inertial-frame direction :math:`{}^\mathcal{N}\hat{h}_\text{ref}`. Alternatively, the direction can be specified as input parameter ``hHat_N``. When this input msg is connected, the input parameter is neglected in favor of the input msg.
    * - ephemerisInMsg
      - :ref:`EphemerisMsgPayload`
      - (optional) Input message containing the inertial position of a celestial object, whose direction with respect to the spacecraft serves as the inertial reference direction :math:`{}^\mathcal{N}\hat{h}_\text{ref}`. This input msg must be provided together with ``transNavInMsg`` to compute the relative position of the celestial object to the spacecraft. If both ``inertialHeadingInMsg`` and ``ephemerisInMsg`` are connected, the inertial reference direction :math:`{}^\mathcal{N}\hat{h}_\text{ref}` is computed according to ``inertialHeadingInMsg``.
    * - transNavInMsg
      - :ref:`NavTransMsgPayload`
      - (optional) Input message containing the inertial position and velocity of the spacecraft. This message must be connected together with ``ephemerisInMsg`` to allow to compute :math:`{}^\mathcal{N}\hat{h}_\text{ref}`.
    * - attRefOutMsg
      - :ref:`AttRefMsgPayload`
      - Output attitude reference message containing reference attitude, reference angular rates and accelerations.


Detailed Module Description
---------------------------
The details of the equations applied by this module can be found in R. Calaon PhD thesis, "Guidance, Control and Momentum Management of Spacecraft with Multiple Pointing Constraints".

Attention must be paid to how these pieces of input information is provided:

  - Input body-frame heading: this can be specified either via the input parameter ``h1Hat_B``, or connecting the input message ``bodyHeadingInMsg``. Specifying the body-frame heading via the input parameter is desirable when such direction does not change over time; vice versa, when the body-frame heading is time varying, this needs to be passed via the ``bodyHeadingInMsg``. When both ``h1Hat_B`` and ``bodyHeadingInMsg`` are provided, the module ignores ``h1Hat_B`` and reads the body-frame direction from the input message.
  - Input inertial-frame heading: this can be specified via the input parameter ``hHat_N``, connecting the message ``inertialHeadingInMsg``, or connecting both the messages ``ephemerisInMsg`` and ``transNavInMsg``. The input parameter ``hHat_N`` is desirable when the inertial heading is fixed in time. The message ``inertialHeadingInMsg`` is needed when the heading direction is time-varying. Finally, providing ``ephemerisInMsg`` and ``transNavInMsg`` allows to compute the inertial heading as the vector difference between the inertial position of a celestial object and the position of the spacecraft: this is useful when the spacecraft needs to point a body-frame heading towards a celestial object. When all of these input messages are connected, the inertial heading is computed from the ``inertialHeadingInMsg``.

Module Assumptions and Limitations
----------------------------------
The limitations of this module are inherent to the geometry of the problem, which determines whether or not all the constraints can be satisfied. This applies on singularities when the Sun Prop Earth angle (SPE) is (0,90,180).


User Guide
----------
The required module configuration is::

    attReference = triad.Triad()
    attReference.modelTag = "triad"
    attReference.a1Hat_B = a1_B
    scSim.AddModelToTaskAddModelToTask(simTaskName, attReference)

The module is configurable with the following parameters:

.. list-table:: Module Parameters
   :widths: 25 25 50
   :header-rows: 1

   * - Parameter
     - Default
     - Description
   * - ``a1Hat_B``
     - [0, 0, 0]
     - solar array drive direction, it must be specified by the user
   * - ``h1Hat_B`` (optional)
     - [0, 0, 0]
     - body-frame heading
   * - ``hHat_N`` (optional)
     - [0, 0, 0]
     - inertial-frame heading
   * - ``celestialBodyInput`` (optional)
     - 0
     - should be set to 1 when the celestial body pointed at is the Sun.
