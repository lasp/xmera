==============================
Sunline Ephemeris Algorithm
==============================

.. list-table::
   :widths: 5 40 10 10
   :header-rows: 0

   * - **Rev**
     - **Change Description**
     - **By**
     - **Date**
   * - 1.0
     - Initial Draft
     - S. SabekZaei
     - 20260105

====================
Module Description
====================

Introduction
============
The ``sunlineEphem`` flight software algorithm is a module that computes the Sun direction vector relative to the spacecraft in the body frame based on ephemeris data.

The module uses the Sun and spacecraft position expressed in the inertial frame, in addition to the spacecraft attitude, to compute the Sun direction in the spacecraft body frame.

This ephemeris-based Sun direction vector can later be used as a reference for other downstream guidance and control modules.

Module Input/Output Messages
============================
The following table lists all the module input and output messages. The msg type contains a link to the message structure definition, while the description provides information on what this message is used for.

.. list-table:: Module I/O Messages
   :widths: 15 30 60
   :header-rows: 1

   * - **Msg Variable Name**
      - **Msg Type**
      - **Description**

   * - ``scAttitudeInMsg``
      - :ref:`NavAttMsgPayload`
      - Input message containing the spacecraft's attitude, where MRPs are used to rotate the Sun vector from the inertial frame to body frame.

   * - ``sunPositionInMsg``
      - :ref:`EphemerisMsgPayload`
      - Input message containing the Sun's position expressed in the inertial frame.

   * - ``scPositionInMsg``
      - :ref:`NavTransMsgPayload`
      - Input message containing the spacecraft's position expressed in the inertial frame.

   * - ``navStateOutMsg``
      - :ref:`NavAttMsgPayload`
      - Output message containing the spacecraft's ephemeris-based Sun direction vector expressed in the body frame.

Algorithm Computation
======================
At every update cycle, the ``sunlineEphem`` module performs the following steps:

- It reads the ephemeris-based Sun position expressed in inertial frame ``rSun`` from ``sunPositionInMsg``.
- It reads the spacecraft position expressed in inertial frame ``rSc`` from ``scPositionInMsg``.
- It then computes the Sun position vector relative to the spacecraft ``r_SB_N``, by subtracting the spacecraft position from the sun position:


  .. math::

    \boldsymbol{r}_{SB,N} = \boldsymbol{r}_{Sun,N} - \boldsymbol{r}_{SC,N}

.. _ModuleIO_sunlineEphem_overview:
.. figure:: /../../src/fswAlgorithms/attDetermination/sunlineEphem/_Documentation/Figures/sunlineEphem_rSB_N.png.jpeg
    :scale: 50 %
    :align: center
    Figure 1: SunlineEphem computes the inertial-frame Sun direction vector :math:`\mathbf{r}_{SB}^{N}`.



- The relative vector is normalized ``r_SB_N_hat`` to obtain the unit direction vector. If :math:`\|\boldsymbol{r}_{SB,N}\| \le \epsilon`, the ``r_SB_N_hat`` will remain a zero vector.
- It later reads the current spacecraft attitude (MRPs) from ``scAttitudeInMsg`` to build a Direction Cosine Matrix (DCM) ``dcm_BN``.
- Using this DCM, the Sun direction unit vector is rotated from inertial to body frame:


  .. math::

    \boldsymbol{\hat r}_{SB,B} =  \mathbf{C}_{BN}(\boldsymbol{\sigma}_{BN}) \, \boldsymbol{\hat r}_{SB,N}

- The resulting vector is normalized ``r_SB_B_hat`` to ensure a unit length. If :math:`\|\boldsymbol{r}_{SB,B}\| \le \epsilon`, the vector will be explicitly set to zero.
- At last, the algorithm writes the three elements of the unit body frame Sun direction vector into the ``vehSunPntBdy`` field of the output navigation message  ``navStateOutMsg``.

Module Assumptions and Limitations
==================================
- It is the user's responsibility to ensure all input position vectors are expressed in the same inertial reference frame.
- Any errors from the ephemeris or navigation data will affect the output accuracy.

Test Description and Success Criteria
=====================================
The Sunline ephemeris unit test is located in ``fswAlgorithms/attDetermination/sunlineEphem/_UnitTest/test_sunlineEphem.py``. This test verifies that the module computes the correct Sun direction vector expressed in the body frame.
In this unit test, the Sun inertial position is set at the origin by `sunData.r_BdyZero_N = [0.0, 0.0, 0.0]`. The spacecraft attitude is set to zero using `vehAttData.sigma_BN = [0.0, 0.0, 0.0]`, which corresponds to an identity rotation. This implies that the inertial and body frames are aligned.
The spacecraft's inertial position is iterated through a set of unit length vectors, with a special case where the spacecraft and sun position vectors are identical.
For each case, the simulation will execute and the Sun direction vector will be recorded.
The truth vectors are defined in the python test, where they are compared with the estimated module output.
The test is considered successful if all the output values of the estimated and truth values are identical within a tolerance of :math:`10^{-12}`, including outputting a zero vector for the special case where the relative vector magnitude is zero.
