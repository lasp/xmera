// SPDX-License-Identifier: ISC
// Copyright (c) 2023, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

#include "flybyODuKF.h"

#include "flybyODuKFAlgorithm.h"

#include <architecture/utilities/eigenSupport.h>
#include <architecture/utilities/macroDefinitions.h>

#include <cassert>
#include <utility>

using filtering::Position;
using filtering::Velocity;
using filtering::flybyODuKF::FilterStateOutput;
using filtering::flybyODuKF::FlybyODuKFAlgorithm;
using filtering::flybyODuKF::HeadingMeasurement;
using filtering::flybyODuKF::ResidualsOutput;

FlybyODuKF::FlybyODuKF() : algorithm(std::make_unique<FlybyODuKFAlgorithm>()) {}
FlybyODuKF::~FlybyODuKF() = default;

void FlybyODuKF::reset(uint64_t currentSimNanos) {
    /*! - Require the input measurement message to be connected. */
    assert(this->opNavHeadingMsg.isLinked());
    this->algorithm->reset();
    this->previousSimNanos = currentSimNanos;
}

void FlybyODuKF::updateState(uint64_t currentSimNanos) {
    double const previousSeconds = static_cast<double>(this->previousSimNanos) * NANO2SEC;
    double const currentSeconds = static_cast<double>(currentSimNanos) * NANO2SEC;

    // Read and enqueue any measurement, then drive the filter over the window
    // with a single call; the algorithm interleaves time and measurement
    // updates from its own queue.
    bool const measurementProcessed = this->readFilterMeasurements(previousSeconds);
    this->algorithm->update(previousSeconds, currentSeconds);

    this->previousSimNanos = currentSimNanos;
    this->writeOutputMessages(currentSimNanos, measurementProcessed);
}

/*! Read the input heading message and, if valid and within the update window,
    marshal it into a HeadingMeasurement and queue it on the algorithm. Returns
    whether a measurement was enqueued. */
bool FlybyODuKF::readFilterMeasurements(double previousSeconds) {
    this->opNavHeadingBuffer = this->opNavHeadingMsg();

    if (!this->opNavHeadingBuffer.valid) return false;
    if (this->opNavHeadingBuffer.timeTag < previousSeconds) return false;

    HeadingMeasurement measurement;
    measurement.timeTag = this->opNavHeadingBuffer.timeTag;
    measurement.rhat_BN_N = cArrayToEigenVector(this->opNavHeadingBuffer.rhat_BN_N);
    measurement.covarN = cArrayToEigenMatrixX(this->opNavHeadingBuffer.covar_N, 3, 3);
    measurement.valid = true;

    this->algorithm->enqueueMeasurement(measurement.timeTag, std::move(measurement));
    return true;
}

/*! Marshal the algorithm's state and residuals into the output messages. */
void FlybyODuKF::writeOutputMessages(uint64_t currentSimNanos, bool measurementProcessed) {
    NavTransMsgPayload navTransOutMsgBuffer{};
    FilterMsgPayload opNavFilterMsgBuffer{};
    FilterResidualsMsgPayload residualsBuffer{};

    FilterStateOutput const filterState = this->algorithm->getState();
    double const timeTag = static_cast<double>(currentSimNanos) * NANO2SEC;

    /*! - Navigation translation output (position/velocity in inertial frame). */
    navTransOutMsgBuffer.timeTag = timeTag;
    eigenMatrixXToCArray(filterState.state.head<3>().eval(), navTransOutMsgBuffer.r_BN_N);
    eigenMatrixXToCArray(filterState.state.tail<3>().eval(), navTransOutMsgBuffer.v_BN_N);

    /*! - Full filter state output. */
    opNavFilterMsgBuffer.timeTag = timeTag;
    opNavFilterMsgBuffer.numberOfStates = FlybyODuKFAlgorithm::N;
    eigenMatrixXToCArray(filterState.state, opNavFilterMsgBuffer.state);
    eigenMatrixXToCArray(filterState.covariance, opNavFilterMsgBuffer.covar);

    /*! - Residuals output, populated only when a measurement was processed. */
    if (measurementProcessed) {
        ResidualsOutput const residuals = this->algorithm->getLastResiduals();
        residualsBuffer.timeTag = timeTag;
        residualsBuffer.valid = residuals.valid;
        residualsBuffer.numberOfObservations = 1;
        residualsBuffer.sizeOfObservations = 3;
        eigenMatrixXToCArray(residuals.observation, residualsBuffer.observation);
        eigenMatrixXToCArray(residuals.preFit, residualsBuffer.preFits);
        eigenMatrixXToCArray(residuals.postFit, residualsBuffer.postFits);
    }

    this->opNavResidualMsg.write(&residualsBuffer, this->moduleID, currentSimNanos);
    this->navTransOutMsg.write(&navTransOutMsgBuffer, this->moduleID, currentSimNanos);
    this->opNavFilterMsg.write(&opNavFilterMsgBuffer, this->moduleID, currentSimNanos);
}

// ---- Configuration forwarders ---------------------------------------------

void FlybyODuKF::setAlpha(double alpha) { this->algorithm->setAlpha(alpha); }
double FlybyODuKF::getAlpha() const { return this->algorithm->getAlpha(); }

void FlybyODuKF::setBeta(double beta) { this->algorithm->setBeta(beta); }
double FlybyODuKF::getBeta() const { return this->algorithm->getBeta(); }

void FlybyODuKF::setUnitConversionFromSItoState(double conversion) {
    this->algorithm->setUnitConversion(conversion);
}
double FlybyODuKF::getUnitConversionFromSItoState() const {
    return this->algorithm->getUnitConversion();
}

void FlybyODuKF::setCentralBodyGravitationParameter(double mu) { this->algorithm->setMu(mu); }
double FlybyODuKF::getCentralBodyGravitationParameter() const { return this->algorithm->getMu(); }

void FlybyODuKF::setMeasurementNoiseScale(double measurementNoiseScale) {
    this->algorithm->setMeasurementNoiseScale(measurementNoiseScale);
}
double FlybyODuKF::getMeasurementNoiseScale() const {
    return this->algorithm->getMeasurementNoiseScale();
}

void FlybyODuKF::setInitialPosition(const Eigen::Vector3d& r_BN_N) {
    auto state = this->algorithm->getInitialState();
    state.set<Position<3>>(r_BN_N);
    this->algorithm->setInitialState(state);
}
Eigen::Vector3d FlybyODuKF::getInitialPosition() const {
    return this->algorithm->getInitialState().get<Position<3>>();
}

void FlybyODuKF::setInitialVelocity(const Eigen::Vector3d& v_BN_N) {
    auto state = this->algorithm->getInitialState();
    state.set<Velocity<3>>(v_BN_N);
    this->algorithm->setInitialState(state);
}
Eigen::Vector3d FlybyODuKF::getInitialVelocity() const {
    return this->algorithm->getInitialState().get<Velocity<3>>();
}

void FlybyODuKF::setInitialCovariance(const Eigen::MatrixXd& covariance) {
    this->algorithm->setInitialCovariance(Eigen::Matrix<double, 6, 6>(covariance));
}
Eigen::MatrixXd FlybyODuKF::getInitialCovariance() const {
    return this->algorithm->getInitialCovariance();
}

void FlybyODuKF::setProcessNoise(const Eigen::MatrixXd& processNoise) {
    this->algorithm->setProcessNoise(Eigen::Matrix<double, 6, 6>(processNoise));
}
Eigen::MatrixXd FlybyODuKF::getProcessNoise() const { return this->algorithm->getProcessNoise(); }
