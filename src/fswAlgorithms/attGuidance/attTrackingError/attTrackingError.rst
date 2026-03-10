=======================
Attitude Tracking Error
=======================

-----------------
Executive Summary
-----------------

The Attitude Tracking Error module reads in current spacecraft attitude, where it should be and a reference correction motion. It then computes the attitude
and rate errors and express it all reference motion in the spacecraft body frame. This module is intended to be the last module in the guidance module chain.
It's output is at the guidance attitude tracking errors relative to a moving reference frame.

----------------------------
Module Input/Output Messages
----------------------------

The following table lists all the module input and output messages:

.. list-table:: Module I/O Messages
    :widths: 25 25 50
    :header-rows: 1

    * - Msg Variable Name
      - Msg Type
      - Description
    * - attGuidOutMsg
      - :ref:`AttGuidMsgPayload`
      - attitude guidance tracking errors output message
    * - attNavInMsg
      - :ref:`NavAttMsgPayload`
      - attitude navigation input message
    * - attRefInMsg
      - :ref:`AttRefMsgPayload`
      - attitude reference input message

----------------
Module Functions
----------------
Below is a list of functions that this simulation module performs

- Reads the incoming attitude navigation and reference attitude message
- Adjust the reference attitude
- Computes the attitude tracking error
- Computes the angular velocity of reference frame relative to inertial frame in body frame components
- Computes the angular velocity error in body frame
- Computes the angular acceleration in body frame
- Writes the guidance module output message

------------------
Module Description
------------------

The following text describes existing function in details and the mathematics behind the `attTrackingError` module.

The `attTrackingError` module computes attitude and rate tracking errors between a commanded reference frame and the spacecraft’s current navigation estimate.
It outputs a guidance payload containing: (1) the attitude error expressed with Modified Rodrigues Parameters (MRPs), (2) the angular velocity tracking
error expressed in body-frame components, and (3) the reference angular velocity and angular acceleration expressed in the body frame.
These outputs are commonly consumed by downstream attitude controllers.

**Parameter**

* :math:`\mathbf\sigma_{R_0R}` : a configurable MRP correction mapping between an original reference frame (R0) and a corrected reference frame (R).
  This allows the module to apply a fixed reference offset to the incoming reference attitude.

**Outputs**

* `AttGuidMsgPayload`:

  * :math:`\mathbf\sigma_{BR}`: MRPs describing the attitude error of (B) relative to (R)
  * :math:`^{B}\mathbf\omega_{BR}`: angular velocity error of (B) relative to (R), expressed in (B)
  * :math:`^{B}\mathbf\omega_{RN}`: reference angular velocity expressed in (B)
  * :math:`^{B}\mathbf\dot\omega_{RN}`: reference angular acceleration expressed in (B)


**1. Reference attitude correction**

The module begins by applying the configured correction :math:`\mathbf\sigma_{R_0R}` to the incoming reference attitude to form an updated reference frame.


**2. Attitude tracking error**

The attitude tracking error :math:`\mathbf\sigma_{BR}` is computed by finding the relative rotation from the corrected reference frame (R)
:math:`\mathbf\sigma_{RN}` to the body frame (B) :math:`\mathbf\sigma_{BN}`.

**3. Reference angular velocity expressed in body components**

Reference angular velocity is provided in inertial components :math:`^{N}\mathbf\omega_{RN}`. For control applications, it is often preferred in body components.
Using the DCM :math:`\mathbf\left[BN]` , the transformation is:

:math:`^{B}\mathbf\omega_{RN}` = :math:`\mathbf\left[BN]`  :math:`^{N}\mathbf\omega_{RN}`

**4. Angular velocity tracking error**

The angular velocity error is computed as the difference between the measured body rate and the reference rate, both expressed in body components:

:math:`^{B}\mathbf\omega_{BR}` = :math:`^{B}\mathbf\omega_{BN}` - :math:`^{B}\mathbf\omega_{RN}`


**5. Reference angular acceleration expressed in body components**

Similarly, the reference angular acceleration is transformed into body components:
:math:`^{B}\mathbf\dot\omega_{RN}` = :math:`\mathbf\left[BN]` :math:`^{N}\mathbf\dot\omega_{RN}`

Overall, `attTrackingError` acts as a kinematic “bridge” between navigation and control: it aligns reference signals into the body frame and provides consistent tracking error
quantities for downstream attitude control laws.



----------------
Test Description
----------------

This unit test confirms correct implementation of the attitude tracking transformations by comparing its outputs to independently computed truth values. Known navigation
and reference attitude states, angular rates, angular accelerations,and a reference correction are provided to the module. The test passes if the computed attitude error,
angular rate error, and reference angular rate and acceleration expressed in the body frame match the expected values within a specified numerical tolerance.


----------
User Guide
----------

The module is configured by::

    attitudeTrackingError = attTrackingError.AttTrackingError()
    attitudeTrackingError.modelTag = "attTrackingError"
    attitudeTrackingError.setSigma_R0R(sigma_R0R)
