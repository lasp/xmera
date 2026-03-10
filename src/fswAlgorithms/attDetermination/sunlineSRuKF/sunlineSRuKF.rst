Executive Summary
-----------------
This module implements a square-root unscented Kalman filter to estimate the direction of the Sun vector in body-frame coordinates using coarse sun sensor (CSS) and gyro measurements. This module is a child class of :ref:`srukfInterface`.

Message Connection Descriptions
-------------------------------
The following table lists all the module input and output messages. The msg type contains a link to the message structure definition, while the description
provides information on what this message is used for.

.. list-table:: Module I/O Messages
    :widths: 25 25 50
    :header-rows: 1

    * - Msg Variable Name
      - Msg Type
      - Description
    * - navAttInMsg
      - :ref:`NavAttMsgPayload`
      - Input gyro message
    * - cssDataInMsg
      - :ref:`CSSArraySensorMsgPayload`
      - Input message containing the direction cosines of the Sun as seen by the CSSs
    * - cssConfigInMsg
      - :ref:`CSSConfigMsgPayload`
      - Input message containing the geometry of the CSS constellation
    * - navAttOutMsg
      - :ref:`NavAttMsgPayload`
      - Output message containing the estimated Sun vector in body-frame coordinates
    * - navAttOutMsgC
      - :ref:`NavAttMsgPayload`
      - Output message containing the estimated Sun vector in body-frame coordinates - C-wrapped
    * - filterOutMsg
      - :ref:`FilterMsgPayload`
      - Output message with the filter estimated state and covariance
    * - filterGyroResOutMsg
      - :ref:`FilterResidualsMsgPayload`
      - Output message containing pre- and post-fit residuals for the gyro measurements
    * - filterCssResOutMsg
      - :ref:`FilterResidualsMsgPayload`
      - Output message containing pre- and post-fit residuals for the CSS measurements



Detailed Module Description
---------------------------
The estimated state of the filter is a 6-dimensional vector combining the sun heading vector in body-frame coordinates and body-frame rates also in body frame coordinates:
The optional bias state models the varying solar intensity which scales the measurements.

.. math::
    \boldsymbol{s} = \left\{ \begin{matrix} {}^\mathcal{B}\boldsymbol{\hat{s}} \\ {}^\mathcal{B}\boldsymbol{\omega} \end{matrix} \right\}.

Dynamics model
++++++++++++++
The sun heading is fixed in inertial coordinates, it only changes in body-frame coordinates due to the motion of the spacecraft. Therefore, the dynamics of the sun heading is only given by the body-frame derivative of the sun heading unit-direction vector. For simplicity, the derivative of the angular rate vector is set to zero in this module. This gives:
If the bias state is present, it has no dynamics.

.. math::
    \boldsymbol{\dot{s}} = \left\{ \begin{matrix} {}^\mathcal{B}\boldsymbol{\hat{s}} \times {}^\mathcal{B}\boldsymbol{\omega} \\ \boldsymbol{0} \end{matrix} \right\}.

Measurement model
+++++++++++++++++
Two types of measurements are processed by this filter: gyro measurements and CSS measurements. For gyro measurements, the gyro rates are mapped directly to the rate component of the state via a :math:`3 \times 3` identity matrix, which constitutes the measurement model.

For the CSS measurement, the measurement model is constituted by the :math:`n \times 3` matrix :math:`[\boldsymbol{H}]`, where :math:`n` is the number of active sun sensors active at that moment. With :math:`{}^\mathcal{B}\boldsymbol{\hat{n}}_i` being the boresight of the :math:`i`-th CSS, the measurement model is given by:

.. math::
    [\boldsymbol{H}] = \left[ \begin{matrix} {}^\mathcal{B}\boldsymbol{\hat{n}}_1 \\ \vdots \\ {}^\mathcal{B}\boldsymbol{\hat{n}}_n \end{matrix} \right].

If a bias state is present and named :math:`b`, and the current state is :math:`\hat{\boldsymbol{s}}`, the predicted measurements will be given by:

.. math::
    yMeas = b[\boldsymbol{H}] \hat{\boldsymbol{s}}


User Guide
----------
The required module configuration is::

    filter_object.setInitialPosition([0.0, 0.0, 1.0])
    filter_object.setInitialVelocity([0.02, -0.005, 0.01])
    filter_object.setInitialBias([0.6])
    filter_object.setInitialCovariance([[0.1, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                                        [0.0, 0.1, 0.0, 0.0, 0.0, 0.0, 0.0],
                                        [0.0, 0.0, 0.1, 0.0, 0.0, 0.0, 0.0],
                                        [0.0, 0.0, 0.0, 0.001, 0.0, 0.0, 0.0],
                                        [0.0, 0.0, 0.0, 0.0, 0.001, 0.0, 0.0],
                                        [0.0, 0.0, 0.0, 0.0, 0.0, 0.001, 0.0],
                                        [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.5]])
    filter_object.setCssMeasurementNoiseStd(0.01)
    filter_object.setGyroMeasurementNoiseStd(0.001)
    filter_object.setBiasLowerBound(1)
    sigmaSun = (1E-6) ** 2
    sigmaRate = (1E-8) ** 2
    sigmaBias = (1E-5) ** 2
    filter_object.setProcessNoise([[sigmaSun, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                                   [0.0, sigmaSun, 0.0, 0.0, 0.0, 0.0, 0.0],
                                   [0.0, 0.0, sigmaSun, 0.0, 0.0, 0.0, 0.0],
                                   [0.0, 0.0, 0.0, sigmaRate, 0.0, 0.0, 0.0],
                                   [0.0, 0.0, 0.0, 0.0, sigmaRate, 0.0, 0.0],
                                   [0.0, 0.0, 0.0, 0.0, 0.0, sigmaRate, 0.0],
                                   [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, sigmaBias]])
