// SPDX-License-Identifier: ISC
// Copyright (c) 2024, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

#include "sunlineSRuKF.h"

void SunlineSRuKF::reset(uint64_t currentSimNanos) {
    this->customReset();
    this->writeOutputMessages(currentSimNanos);
    this->previousSimNanos = currentSimNanos;
}

void SunlineSRuKF::updateState(const uint64_t currentSimNanos) {
    this->readFilterMeasurements();

    this->measurements.applyToFilter(
        this->srukf,
        (double)this->previousSimNanos * NANO2SEC,
        (double)currentSimNanos * NANO2SEC
    );
    this->previousSimNanos = currentSimNanos;

    this->writeOutputMessages(currentSimNanos);
    this->customFinalizeUpdate();
}

/*! Reset the sunline filter to an initial state and
 initializes the internal estimation matrices.
 @return void
 @param currentSimNanos The clock time at which the function was called (nanoseconds)
 */
void SunlineSRuKF::customReset() {
    this->srukf.dynamics = &SunlineSRuKF::stateDerivative;
    /*! - Check if the required messages have been connected */
    assert(this->cssDataInMsg.isLinked());
    assert(this->cssConfigInMsg.isLinked());
    assert(this->navAttInMsg.isLinked());

    /*! read in CSS configuration message */
    this->cssConfigInputBuffer = this->cssConfigInMsg();
}

/*! Normalize the updated sunline estimate
 @return void
 @param currentSimNanos The clock time at which the function was called (nanoseconds)
 */
void SunlineSRuKF::customFinalizeUpdate() {
    PositionState<Eigen::Dynamic> heading;
    heading.setValues(this->srukf.state.getPositionStates().normalized());
    this->srukf.state.setPosition(heading);

    if (this->srukf.state.hasBias()) {
        BiasState<Eigen::Dynamic> bias;
        if (this->srukf.state.getBiasStates().value() < this->biasLowerBound) {
            Eigen::VectorXd lowerSaturateBias(1);
            lowerSaturateBias(0) = this->biasLowerBound;
            bias.setValues(lowerSaturateBias);
            this->srukf.state.setBias(bias);
        } else if (this->srukf.state.getBiasStates().value() > this->biasUpperBound) {
            Eigen::VectorXd upperSaturateBias(1);
            upperSaturateBias(0) = this->biasUpperBound;
            bias.setValues(upperSaturateBias);
            this->srukf.state.setBias(bias);
        }
    }
}

/*! Read the message containing the measurement data.
 It updates class variables relating to measurement data including validity and time tags.
 @return void
 */
void SunlineSRuKF::writeOutputMessages(uint64_t currentSimNanos) {
    NavAttMsgPayload navAttOutMsgBuffer{};
    FilterMsgPayload filterMsgBuffer{};
    FilterResidualsMsgPayload filterGyroResMsgBuffer{};
    FilterResidualsMsgPayload filterCssResMsgBuffer{};

    /*! - Write the sunline estimate into the copy of the navigation message structure*/
    eigenMatrixXToCArray(this->srukf.state.getPositionStates(), navAttOutMsgBuffer.vehSunPntBdy);

    /*! - Populate the filter states output buffer and write the output message*/
    filterMsgBuffer.timeTag = (double)this->previousSimNanos * NANO2SEC;
    eigenMatrixXToCArray(this->srukf.state.returnValues(), filterMsgBuffer.state);
    eigenMatrixXToCArray(this->srukf.xBar.returnValues(), filterMsgBuffer.stateError);
    eigenMatrixXToCArray(this->srukf.covar, filterMsgBuffer.covar);
    filterMsgBuffer.numberOfStates = this->srukf.state.size();

    for (
        auto it = this->measurements.popEarliest();
        it.has_value();
        it = this->measurements.popEarliest()
    ) {
        auto& measurement = it.value().second;

        switch (measurement.type) {
        case SunlineSRuKFMeasurementType::Gyro:
            filterGyroResMsgBuffer.valid = true;
            filterGyroResMsgBuffer.numberOfObservations = 1;
            filterGyroResMsgBuffer.sizeOfObservations = measurement.observation.size();
            eigenMatrixXToCArray(measurement.observation, filterGyroResMsgBuffer.observation);
            eigenMatrixXToCArray(measurement.postFitResiduals, filterGyroResMsgBuffer.postFits);
            eigenMatrixXToCArray(measurement.preFitResiduals, filterGyroResMsgBuffer.preFits);
            break;

        case SunlineSRuKFMeasurementType::Css:
            filterCssResMsgBuffer.valid = true;
            filterCssResMsgBuffer.numberOfObservations = 1;
            filterCssResMsgBuffer.sizeOfObservations = measurement.observation.size();
            eigenMatrixXToCArray(measurement.observation, filterCssResMsgBuffer.observation);
            eigenMatrixXToCArray(measurement.postFitResiduals, filterCssResMsgBuffer.postFits);
            eigenMatrixXToCArray(measurement.preFitResiduals, filterCssResMsgBuffer.preFits);
            break;
        }
    }

    this->navAttOutMsg.write(&navAttOutMsgBuffer, this->moduleID, currentSimNanos);
    this->filterOutMsg.write(&filterMsgBuffer, this->moduleID, currentSimNanos);
    this->filterCssResOutMsg.write(&filterCssResMsgBuffer, this->moduleID, currentSimNanos);
    this->filterGyroResOutMsg.write(&filterGyroResMsgBuffer, this->moduleID, currentSimNanos);
}

/*! Read the rate gyro input message
 @return void
 */
void SunlineSRuKF::readGyroMeasurements() {
    /*! Read rate gyro measurements */
    NavAttMsgPayload navAttInputBuffer = this->navAttInMsg();
    if (navAttInputBuffer.timeTag < (double)this->previousSimNanos * NANO2SEC) return;

    Eigen::MatrixXd I = Eigen::Matrix3d::Identity();

    auto gyroMeasurement = SunlineSRuKFMeasurementModel();
    gyroMeasurement.type = SunlineSRuKFMeasurementType::Gyro;
    gyroMeasurement.observation = cArrayToEigenVector(navAttInputBuffer.omega_BN_B);
    gyroMeasurement.measNoise = this->measNoiseScaling * pow(this->gyroMeasNoiseStd, 2) * I;

    /*! - Read measurement and cholesky decomposition its noise*/
    this->measurements.enqueue(navAttInputBuffer.timeTag, std::move(gyroMeasurement));
}

/*! Read the coarse sun sensor input message
 @return void
 */
void SunlineSRuKF::readCssMeasurements() {
    /*! Read css data msg */
    CSSArraySensorMsgPayload cssInputBuffer = this->cssDataInMsg();

    /*! - Zero the observed active CSS count */
    this->numActiveCss = 0;

    /*! - Define the linear model matrix H */
    Eigen::MatrixXd hMatrix;
    Eigen::VectorXd cssObservation;
    bool validObservation = false;
    double observationTimeTag = 0;

    /*! - Loop over the maximum number of sensors to check for good measurements */
    /*! -# Isolate if measurement is good */
    /*! -# Set body vector for this measurement */
    /*! -# Get measurement value into observation vector */
    /*! -# Set inverse noise matrix */
    /*! -# increase the number of valid observations */
    /*! -# Otherwise just continue */
    for (uint32_t i = 0; i < this->cssConfigInputBuffer.nCSS; ++i) {
        if (cssInputBuffer.CosValue[i] <= this->sensorUseThresh) continue;

        cssObservation.conservativeResize(this->numActiveCss + 1);
        cssObservation(this->numActiveCss) = cssInputBuffer.CosValue[i];

        hMatrix.conservativeResize(this->numActiveCss + 1, 3);
        for (int j = 0; j < 3; ++j) {
            hMatrix(this->numActiveCss, j) =
                this->cssConfigInputBuffer.cssVals[i].CBias * this->cssConfigInputBuffer.cssVals[i].nHat_B[j];
        }

        validObservation = true;
        observationTimeTag = cssInputBuffer.timeTag;
        this->numActiveCss += 1;
    }

    if (!validObservation) return;
    if (observationTimeTag < (double)this->previousSimNanos * NANO2SEC) return;

    /*! - Read measurement and cholesky decomposition its noise*/
    Eigen::MatrixXd I(this->numActiveCss, this->numActiveCss);
    I.setIdentity();

    auto cssMeasurement = SunlineSRuKFMeasurementModel();
    cssMeasurement.type = SunlineSRuKFMeasurementType::Css;
    cssMeasurement.observation = cssObservation;
    cssMeasurement.hMatrix = hMatrix;
    cssMeasurement.measNoise =
        ( this->measNoiseScaling
        * pow(this->cssMeasNoiseStd, 2)
        * I);

    this->measurements.enqueue(observationTimeTag, std::move(cssMeasurement));
}

/*! Read the message containing the measurement data.
 * It updates class variables relating to measurement data including validity and time tags.
 @return void
 */
void SunlineSRuKF::readFilterMeasurements() {
    /*! zero filter measurement index */
    this->filterMeasurement = 0;

    this->readGyroMeasurements();
    this->readCssMeasurements();
}

/*! Define the equations of motion for the filter dynamics
    @param double time
    @return FilterStateVector inputState
    @return FilterStateVector outputState
    */
FilterStateVector SunlineSRuKF::stateDerivative(const double t, const FilterStateVector& state) {
    FilterStateVector XDot;
    /*! Implement propagation with rate derivatives set to zero */
    Eigen::Vector3d sHat = state.getPositionStates();
    Eigen::Vector3d omega = state.getVelocityStates();

    PositionState<Eigen::Dynamic> xDotPosition;
    VelocityState<Eigen::Dynamic> xDotVelocity;

    xDotPosition.setValues(sHat.cross(omega));
    xDotVelocity.setValues(Eigen::VectorXd::Zero(3));

    XDot.setPosition(xDotPosition);
    XDot.setVelocity(xDotVelocity);

    if (state.hasBias()) {
        BiasState<Eigen::Dynamic> xDotBias;
        xDotBias.setValues(Eigen::VectorXd::Zero(1));
        XDot.setBias(xDotBias);
    }

    return XDot;
}

/*! Set the CSS measurement noise
    @param double cssMeasurementNoise
    @return void
    */
void SunlineSRuKF::setCssMeasurementNoiseStd(const double cssMeasurementNoiseStd) {
    this->cssMeasNoiseStd = cssMeasurementNoiseStd;
}

/*! Set the gyro measurement noise
    @param double gyroMeasurementNoise
    @return void
    */
void SunlineSRuKF::setGyroMeasurementNoiseStd(const double gyroMeasurementNoiseStd) {
    this->gyroMeasNoiseStd = gyroMeasurementNoiseStd;
}

/*! Set the filter measurement noise scale factor if desirable
    @param double measurementNoiseScale
    @return void
    */
void SunlineSRuKF::setMeasurementNoiseScale(const double measurementNoiseScale) {
    this->measNoiseScaling = measurementNoiseScale;
}

/*! Get the CSS measurement noise
    @param double cssMeasurementNoise
    @return void
    */
double SunlineSRuKF::getCssMeasurementNoiseStd() const { return this->cssMeasNoiseStd; }

/*! Get the gyro measurement noise
    @param double gyroMeasurementNoise
    @return void
    */
double SunlineSRuKF::getGyroMeasurementNoiseStd() const { return this->gyroMeasNoiseStd; }

/*! Get the filter measurement noise scale factor
    @return double measurementNoiseScale
    */
double SunlineSRuKF::getMeasurementNoiseScale() const { return this->measNoiseScaling; }

/*! Set the threshold value to accept a css measurement
    @param double threshold
    @return void
    */
void SunlineSRuKF::setSensorThreshold(double threshold) { this->sensorUseThresh = threshold; }

/*! Get the threshold value to accept a css measurement
    @return double threshold
    */
double SunlineSRuKF::getSensorThreshold() const { return this->sensorUseThresh; }

/*! Set the bias upper bound value it is not allowed to exceed
    @param double biasUpperBound
    */
void SunlineSRuKF::setBiasUpperBound(double biasUpperBound) { this->biasUpperBound = biasUpperBound; }

/*! Get the bias upper bound value it is not allowed to exceed
    @return double biasUpperBound
    */
double SunlineSRuKF::getBiasUpperBound() const { return this->biasUpperBound; }

/*! Set the bias lower bound value it is not allowed to subceed
    @param double biasUpperBound
    */
void SunlineSRuKF::setBiasLowerBound(double biasLowerBound) { this->biasLowerBound = biasLowerBound; }

/*! Get the bias lower bound value it is not allowed to subceed
    @return double biasUpperBound
    */
double SunlineSRuKF::getBiasLowerBound() const { return this->biasLowerBound; }
