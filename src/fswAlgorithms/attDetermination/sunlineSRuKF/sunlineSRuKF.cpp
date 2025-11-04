// SPDX-License-Identifier: ISC
// Copyright (c) 2024, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

#include "sunlineSRuKF.h"

void SunlineSRuKF::reset(uint64_t currentSimNanos) {
    this->customReset();
    this->writeOutputMessages(currentSimNanos);
}

void SunlineSRuKF::updateState(const uint64_t currentSimNanos) {
    this->readFilterMeasurements();
    this->srukf.updateState(currentSimNanos, this->measurements);
    this->writeOutputMessages(currentSimNanos);
    this->customFinalizeUpdate();
}

/*! Reset the sunline filter to an initial state and
 initializes the internal estimation matrices.
 @return void
 @param currentSimNanos The clock time at which the function was called (nanoseconds)
 */
void SunlineSRuKF::customReset() {
    this->srukf.setFilterDynamics(SunlineSRuKF::stateDerivative);
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
    PositionState heading;
    heading.setValues(this->srukf.state.getPositionStates().normalized());
    this->srukf.state.setPosition(heading);

    if (this->srukf.state.hasBias()) {
        BiasState bias;
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
    filterMsgBuffer.timeTag = this->srukf.previousFilterTimeTag;
    eigenMatrixXToCArray(this->srukf.state.returnValues(), filterMsgBuffer.state);
    eigenMatrixXToCArray(this->srukf.xBar.returnValues(), filterMsgBuffer.stateError);
    eigenMatrixXToCArray(this->srukf.covar, filterMsgBuffer.covar);
    filterMsgBuffer.numberOfStates = this->srukf.state.size();

    int i = 0;
    for (auto optionalMeasurement : this->measurements) {
        if (optionalMeasurement.has_value() && optionalMeasurement->getMeasurementName() == "gyro") {
            auto measurement = MeasurementModel();
            measurement = optionalMeasurement.value();
            filterGyroResMsgBuffer.valid = true;
            filterGyroResMsgBuffer.numberOfObservations = 1;
            filterGyroResMsgBuffer.sizeOfObservations = measurement.size();
            eigenMatrixXToCArray(measurement.getObservation(), filterGyroResMsgBuffer.observation);
            eigenMatrixXToCArray(measurement.getPostFitResiduals(), filterGyroResMsgBuffer.postFits);
            eigenMatrixXToCArray(measurement.getPreFitResiduals(), filterGyroResMsgBuffer.preFits);
        } else if (optionalMeasurement.has_value() && optionalMeasurement->getMeasurementName() == "css") {
            auto measurement = MeasurementModel();
            measurement = optionalMeasurement.value();
            filterCssResMsgBuffer.valid = true;
            filterCssResMsgBuffer.numberOfObservations = 1;
            filterCssResMsgBuffer.sizeOfObservations = measurement.size();
            eigenMatrixXToCArray(measurement.getObservation(), filterCssResMsgBuffer.observation);
            eigenMatrixXToCArray(measurement.getPostFitResiduals(), filterCssResMsgBuffer.postFits);
            eigenMatrixXToCArray(measurement.getPreFitResiduals(), filterCssResMsgBuffer.preFits);
        }
        this->measurements[i].reset();
        i += 1;
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

    if (navAttInputBuffer.timeTag >= this->srukf.previousFilterTimeTag) {
        auto gyroMeasurements = MeasurementModel();
        gyroMeasurements.setValidity(true);
        gyroMeasurements.setMeasurementName("gyro");
        gyroMeasurements.setTimeTag(navAttInputBuffer.timeTag);
        gyroMeasurements.setObservation(cArrayToEigenVector(navAttInputBuffer.omega_BN_B));
        gyroMeasurements.setMeasurementModel(MeasurementModel::velocityStates);
        Eigen::MatrixXd I = Eigen::Matrix3d::Identity();
        gyroMeasurements.setMeasurementNoise(this->srukf.getMeasurementNoiseScale() * pow(this->gyroMeasNoiseStd, 2) * I);

        /*! - Read measurement and cholesky decomposition its noise*/
        this->measurements[this->filterMeasurement] = gyroMeasurements;
        this->filterMeasurement += 1;
    }
}

/*! Read the coarse sun sensor input message
 @return void
 */
void SunlineSRuKF::readCssMeasurements() {
    /*! Read css data msg */
    CSSArraySensorMsgPayload cssInputBuffer = this->cssDataInMsg();
    auto cssMeasurements = MeasurementModel();
    cssMeasurements.setValidity(false);

    /*! - Zero the observed active CSS count */
    this->numActiveCss = 0;

    /*! - Define the linear model matrix H */
    Eigen::MatrixXd hMatrix;
    Eigen::VectorXd cssObservation;

    /*! - Loop over the maximum number of sensors to check for good measurements */
    /*! -# Isolate if measurement is good */
    /*! -# Set body vector for this measurement */
    /*! -# Get measurement value into observation vector */
    /*! -# Set inverse noise matrix */
    /*! -# increase the number of valid observations */
    /*! -# Otherwise just continue */
    for (uint32_t i = 0; i < this->cssConfigInputBuffer.nCSS; ++i) {
        if (cssInputBuffer.CosValue[i] > this->sensorUseThresh) {
            cssMeasurements.setValidity(true);
            cssObservation.conservativeResize(this->numActiveCss + 1);
            cssObservation(this->numActiveCss) = cssInputBuffer.CosValue[i];
            hMatrix.conservativeResize(this->numActiveCss + 1, 3);
            for (int j = 0; j < 3; ++j) {
                hMatrix(this->numActiveCss, j) =
                    this->cssConfigInputBuffer.cssVals[i].CBias * this->cssConfigInputBuffer.cssVals[i].nHat_B[j];
            }
            cssMeasurements.setTimeTag(cssInputBuffer.timeTag);
            this->numActiveCss += 1;
        }
    }

    std::function<const Eigen::VectorXd(const FilterStateVector)> linearModel =
        [hMatrix](const FilterStateVector& state) {
            Eigen::VectorXd observed = hMatrix * state.getPositionStates();
            if (state.hasBias()) {
                observed = observed * state.getBiasStates().value();
            }
            return observed;
        };

    if (cssMeasurements.getValidity() && cssMeasurements.getTimeTag() >= this->srukf.previousFilterTimeTag) {
        /*! - Read measurement and cholesky decomposition its noise*/
        Eigen::MatrixXd I(this->numActiveCss, this->numActiveCss);
        I.setIdentity();
        cssMeasurements.setMeasurementNoise(this->srukf.getMeasurementNoiseScale() * pow(this->cssMeasNoiseStd, 2) * I);
        cssMeasurements.setObservation(cssObservation);
        cssMeasurements.setMeasurementModel(linearModel);
        cssMeasurements.setMeasurementName("css");
        this->measurements[this->filterMeasurement] = cssMeasurements;
        this->filterMeasurement += 1;
    }
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

    PositionState xDotPosition;
    VelocityState xDotVelocity;

    xDotPosition.setValues(sHat.cross(omega));
    xDotVelocity.setValues(Eigen::VectorXd::Zero(3));

    XDot.setPosition(xDotPosition);
    XDot.setVelocity(xDotVelocity);

    if (state.hasBias()) {
        BiasState xDotBias;
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
