#include "sunlineSRuKFTemplated.hpp"
#include "SRUnscentedKalmanFilter.hpp"
#include "GyroModel.hpp"
#include "Measurement.hpp"

#include <Eigen/Core>

static const int STATE_DIM = 6;

/*! Define the equations of motion for the filter dynamics
    @param double time
    @return FilterStateVector inputState
    @return FilterStateVector outputState
    */
static State<6> stateDerivative(double t, const State<6>& state) {
    State<6> XDot;
    /*! Implement propagation with rate derivatives set to zero */
    Eigen::Vector3d sHat = state.x.segment(0, 3);
    Eigen::Vector3d omega = state.x.segment(3, 3);

    Eigen::Vector3d xDotPosition = sHat.cross(omega);
    Eigen::Vector3d xDotVelocity = Eigen::Vector3d::Zero(3);

    XDot.x.segment(0, 3) = xDotPosition;
    XDot.x.segment(3, 3) = xDotVelocity;

    //    if (state.hasBias()) {
    //        BiasState<double, 3> xDotBias;
    //        xDotBias.setValues(Eigen::VectorXd::Zero(1));
    //        XDot.setBias(xDotBias);
    //    }

    return XDot;
};

SunlineSRuKFTemplated::SunlineSRuKFTemplated() {
    //    this->ukf = SRUnscentedKalmanFilter<6>();
    // std::array<STATE_DIM> measurements;
    // GyroModel gyro;
    // Eigen::Matrix<double, 3, 1> gyroZ;
    // gyroZ << 0.1, 0.2, 0.3;
    //
    //
    // measurements[0] = std::make_unique<Measurement<3, STATE_DIM, GyroModel>>(gyro, gyroZ);

    ukf.setDynamicsModel(stateDerivative);
}
/*! Reset the sunline filter to an initial state and
 initializes the internal estimation matrices.
 @return void
 @param currentSimNanos The clock time at which the function was called (nanoseconds)
 */
void SunlineSRuKFTemplated::reset(uint64_t currentSimNanos) {
    //    this->setFilterDynamics(SunlineSRuKFTemplated::stateDerivative);
    /*! - Check if the required messages have been connected */
    assert(this->cssDataInMsg.isLinked());
    assert(this->cssConfigInMsg.isLinked());
    assert(this->navAttInMsg.isLinked());

    /*! read in CSS configuration message */
    this->cssConfigInputBuffer = this->cssConfigInMsg();
}

void SunlineSRuKFTemplated::updateState(uint64_t currentSimNanos) {
    // Read all available measurements
    this->readFilterMeasurements();

    GyroModel gyro;
    Eigen::Matrix<double, 3, 1> gyroZ;
    gyroZ << 0.1, 0.2, 0.3;

    std::array<std::unique_ptr<IMeasurement<STATE_DIM>>, 5> measurements;
    measurements[0] = std::make_unique<Measurement<3, STATE_DIM, GyroModel>>(gyro, gyroZ);

    ukf.updateState(currentSimNanos, measurements);

    const auto& state = ukf.getState();
    std::cout << "Post-update state x:\n" << state.x << "\n";
    std::cout << "Post-update covariance P:\n" << state.P << "\n";
    this->writeOutputMessages(currentSimNanos);
}

/*! Read the message containing the measurement data.
 It updates class variables relating to measurement data including validity and time tags.
 @return void
 */
void SunlineSRuKFTemplated::writeOutputMessages(uint64_t currentSimNanos) {
    NavAttMsgPayload navAttOutMsgBuffer{};
    FilterMsgPayload filterMsgBuffer{};
    FilterResidualsMsgPayload filterGyroResMsgBuffer{};
    FilterResidualsMsgPayload filterCssResMsgBuffer{};

    /*! - Write the sunline estimate into the copy of the navigation message structure*/
    //    eigenMatrixXd2CArray(this->ukf->state.getPositionStates(), navAttOutMsgBuffer.vehSunPntBdy);

    /*! - Populate the filter states output buffer and write the output message*/
    //    filterMsgBuffer.timeTag = this->filter->previousFilterTimeTag;
    //    eigenMatrixXd2CArray(this->filter->state.returnValues(), filterMsgBuffer.state);
    //    eigenMatrixXd2CArray(this->filter->xBar.returnValues(), filterMsgBuffer.stateError);
    //    eigenMatrixXd2CArray(this->filter->covar, filterMsgBuffer.covar);
    //    filterMsgBuffer.numberOfStates = this->filter->state.size();
    //
    //    int i = 0;
    //    for (auto optionalMeasurement : this->filter->measurements) {
    //        if (optionalMeasurement.has_value() && optionalMeasurement->getMeasurementName() == "gyro") {
    //            auto measurement = MeasurementModel();
    //            measurement = optionalMeasurement.value();
    //            filterGyroResMsgBuffer.valid = true;
    //            filterGyroResMsgBuffer.numberOfObservations = 1;
    //            filterGyroResMsgBuffer.sizeOfObservations = measurement.size();
    //            eigenMatrixXd2CArray(measurement.getObservation(), &filterGyroResMsgBuffer.observation[0]);
    //            eigenMatrixXd2CArray(measurement.getPostFitResiduals(), &filterGyroResMsgBuffer.postFits[0]);
    //            eigenMatrixXd2CArray(measurement.getPreFitResiduals(), &filterGyroResMsgBuffer.preFits[0]);
    //        } else if (optionalMeasurement.has_value() && optionalMeasurement->getMeasurementName() == "css") {
    //            auto measurement = MeasurementModel();
    //            measurement = optionalMeasurement.value();
    //            filterCssResMsgBuffer.valid = true;
    //            filterCssResMsgBuffer.numberOfObservations = 1;
    //            filterCssResMsgBuffer.sizeOfObservations = measurement.size();
    //            eigenMatrixXd2CArray(measurement.getObservation(), &filterCssResMsgBuffer.observation[0]);
    //            eigenMatrixXd2CArray(measurement.getPostFitResiduals(), &filterCssResMsgBuffer.postFits[0]);
    //            eigenMatrixXd2CArray(measurement.getPreFitResiduals(), &filterCssResMsgBuffer.preFits[0]);
    //        }
    //        this->filter->measurements[i].reset();
    //        i += 1;
    //    }
    //
    //    this->navAttOutMsg.write(&navAttOutMsgBuffer, this->moduleID, currentSimNanos);
    //    this->filterOutMsg.write(&filterMsgBuffer, this->moduleID, currentSimNanos);
    //    this->filterCssResOutMsg.write(&filterCssResMsgBuffer, this->moduleID, currentSimNanos);
    //    this->filterGyroResOutMsg.write(&filterGyroResMsgBuffer, this->moduleID, currentSimNanos);
}

/*! Read the rate gyro input message
 @return void
 */
void SunlineSRuKFTemplated::readGyroMeasurements() {
    /*! Read rate gyro measurements */
    //    NavAttMsgPayload navAttInputBuffer = this->navAttInMsg();
    //
    //    if (navAttInputBuffer.timeTag >= this->filter->previousFilterTimeTag) {
    //        auto gyroMeasurements = MeasurementModel();
    //        gyroMeasurements.setValidity(true);
    //        gyroMeasurements.setMeasurementName("gyro");
    //        gyroMeasurements.setTimeTag(navAttInputBuffer.timeTag);
    //        gyroMeasurements.setObservation(cArray2EigenVector3d(navAttInputBuffer.omega_BN_B));
    //        gyroMeasurements.setMeasurementModel(MeasurementModel::velocityStates);
    //        Eigen::MatrixXd I = Eigen::Matrix3d::Identity();
    //        gyroMeasurements.setMeasurementNoise(this->filter->measNoiseScaling * pow(this->gyroMeasNoiseStd, 2) * I);
    //
    //        /*! - Read measurement and cholesky decomposition its noise*/
    //        this->filter->measurements[this->filterMeasurement] = gyroMeasurements;
    //        this->filterMeasurement += 1;
    //    }
}

/*! Read the coarse sun sensor input message
 @return void
 */
void SunlineSRuKFTemplated::readCssMeasurements() {
    /*! Read css data msg */
    //    CSSArraySensorMsgPayload cssInputBuffer = this->cssDataInMsg();
    //    auto cssMeasurements = MeasurementModel<StateConfig>();
    //    cssMeasurements.setValidity(false);
    //
    //    /*! - Define the linear model matrix H */
    //    Eigen::Matrix<ScalarT, MAX_NUM_CSS_SENSORS, 3> hMatrix;
    //    Eigen::Vector<ScalarT, MAX_NUM_CSS_SENSORS> cssObservation;

    /*! - Loop over the maximum number of sensors to check for good measurements */
    /*! -# Isolate if measurement is good */
    /*! -# Set body vector for this measurement */
    /*! -# Get measurement value into observation vector */
    /*! -# Set inverse noise matrix */
    /*! -# increase the number of valid observations */
    /*! -# Otherwise just continue */
    /*! - Zero the observed active CSS count */
    //    this->numActiveCss = 0;
    //    for (uint32_t i = 0; i < this->cssConfigInputBuffer.nCSS; ++i) {
    //        if (cssInputBuffer.CosValue[i] > this->sensorUseThresh) {
    //            cssMeasurements.setValidity(true);
    ////            cssObservation.conservativeResize(this->numActiveCss + 1);
    //            cssObservation(this->numActiveCss) = cssInputBuffer.CosValue[i];
    ////            hMatrix.conservativeResize(this->numActiveCss + 1, 3);
    //            for (int j = 0; j < 3; ++j) {
    //                hMatrix(this->numActiveCss, j) =
    //                    this->cssConfigInputBuffer.cssVals[i].CBias * this->cssConfigInputBuffer.cssVals[i].nHat_B[j];
    //            }
    //            cssMeasurements.setTimeTag(cssInputBuffer.timeTag);
    //            this->numActiveCss += 1;
    //        }
    //    }
    //
    //    std::function<const Eigen::Matrix<StateConfig::ScalarT (const FilterStateVector)> linearModel =
    //        [hMatrix](const FilterStateVector &state) {
    //            Eigen::VectorXd observed = hMatrix * state.getPositionStates();
    //            if (state.hasBias()) {
    //                observed = observed * state.getBiasStates().value();
    //            }
    //            return observed;
    //        };
    //
    //    if (cssMeasurements.getValidity() && cssMeasurements.getTimeTag() >= this->filter->previousFilterTimeTag) {
    //        /*! - Read measurement and cholesky decomposition its noise*/
    //        Eigen::MatrixXd I(this->numActiveCss, this->numActiveCss);
    //        I.setIdentity();
    //        cssMeasurements.setMeasurementNoise(this->filter->measNoiseScaling * pow(this->cssMeasNoiseStd, 2) * I);
    //        cssMeasurements.setObservation(cssObservation);
    //        cssMeasurements.setMeasurementModel(linearModel);
    //        cssMeasurements.setMeasurementName("css");
    //        this->filter->measurements[this->filterMeasurement] = cssMeasurements;
    //        this->filterMeasurement += 1;
    //    }
}

/*! Read the message containing the measurement data.
 * It updates class variables relating to measurement data including validity and time tags.
 @return void
 */
void SunlineSRuKFTemplated::readFilterMeasurements() {
    /*! zero filter measurement index */
    //    this->filterMeasurement = 0;

    this->readGyroMeasurements();
    this->readCssMeasurements();
}

/*! Normalize the updated sunline estimate
 @return void
 @param currentSimNanos The clock time at which the function was called (nanoseconds)
 */
// void SunlineSRuKFTemplated::customFinalizeUpdate() {
//     PositionState<double, 3> heading;
//     heading.setValues(this->filter->state.getPositionStates().normalized());
//     this->filter->state.setPosition(heading);
//
//     if (this->filter->state.hasBias()) {
//         BiasState<double, 3> bias;
//         if (this->filter->state.getBiasStates().value() < this->biasLowerBound) {
//             Eigen::VectorXd lowerSaturateBias(1);
//             lowerSaturateBias(0) = this->biasLowerBound;
//             bias.setValues(lowerSaturateBias);
//             this->filter->state.setBias(bias);
//         } else if (this->filter->state.getBiasStates().value() > this->biasUpperBound) {
//             Eigen::VectorXd upperSaturateBias(1);
//             upperSaturateBias(0) = this->biasUpperBound;
//             bias.setValues(upperSaturateBias);
//             this->filter->state.setBias(bias);
//         }
//     }
// }

/*! Set the CSS measurement noise
    @param double cssMeasurementNoise
    @return void
    */
void SunlineSRuKFTemplated::setCssMeasurementNoiseStd(const double cssMeasurementNoiseStd) {
    this->cssMeasNoiseStd = cssMeasurementNoiseStd;
}

/*! Set the gyro measurement noise
    @param double gyroMeasurementNoise
    @return void
    */
void SunlineSRuKFTemplated::setGyroMeasurementNoiseStd(const double gyroMeasurementNoiseStd) {
    this->gyroMeasNoiseStd = gyroMeasurementNoiseStd;
}

/*! Get the CSS measurement noise
    @param double cssMeasurementNoise
    @return void
    */
double SunlineSRuKFTemplated::getCssMeasurementNoiseStd() const { return this->cssMeasNoiseStd; }

/*! Get the gyro measurement noise
    @param double gyroMeasurementNoise
    @return void
    */
double SunlineSRuKFTemplated::getGyroMeasurementNoiseStd() const { return this->gyroMeasNoiseStd; }

/*! Set the threshold value to accept a css measurement
    @param double threshold
    @return void
    */
void SunlineSRuKFTemplated::setSensorThreshold(double threshold) { this->sensorUseThresh = threshold; }

/*! Get the threshold value to accept a css measurement
    @return double threshold
    */
double SunlineSRuKFTemplated::getSensorThreshold() const { return this->sensorUseThresh; }

/*! Set the bias upper bound value it is not allowed to exceed
    @param double biasUpperBound
    */
void SunlineSRuKFTemplated::setBiasUpperBound(double biasUpperBound) { this->biasUpperBound = biasUpperBound; }

/*! Get the bias upper bound value it is not allowed to exceed
    @return double biasUpperBound
    */
double SunlineSRuKFTemplated::getBiasUpperBound() const { return this->biasUpperBound; }

/*! Set the bias lower bound value it is not allowed to subceed
    @param double biasUpperBound
    */
void SunlineSRuKFTemplated::setBiasLowerBound(double biasLowerBound) { this->biasLowerBound = biasLowerBound; }

/*! Get the bias lower bound value it is not allowed to subceed
    @return double biasUpperBound
    */
double SunlineSRuKFTemplated::getBiasLowerBound() const { return this->biasLowerBound; }
