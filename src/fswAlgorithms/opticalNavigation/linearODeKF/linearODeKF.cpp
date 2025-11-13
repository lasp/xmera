// SPDX-License-Identifier: ISC
// Copyright (c) 2024, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

#include "linearODeKF.h"

void LinearODeKF::reset(uint64_t currentSimNanos) {
    this->customReset();
    this->ekf.reset();
    this->previousSimNanos = currentSimNanos;
}

void LinearODeKF::updateState(uint64_t currentSimNanos) {
    this->readFilterMeasurements();

    this->measurements.applyToFilter(
        this->ekf,
        (double)this->previousSimNanos * NANO2SEC,
        (double)currentSimNanos * NANO2SEC
    );
    this->previousSimNanos = currentSimNanos;

    this->writeOutputMessages(currentSimNanos);
}

/*! Reset the flyby OD filter to an initial state and initializes the internal estimation matrices.
 @return void
 @param currentSimNanos The clock time at which the function was called (nanoseconds)
 */
void LinearODeKF::customReset() {
    /*! - Check if the required message has not been connected */
    assert(this->opNavHeadingMsg.isLinked());

    /*! - Set the filter dynamics (linear) */
    this->ekf.dynamics = [](double t, const EkfStateVector<3>& state) -> EkfStateVector<3> {
        return {
            .position = state.velocity,
            .velocity = Eigen::Vector3d::Zero(),
        };
    };

    this->ekf.dynamicsTransitionMatrix = [](double t, const EkfStateVector<3>& state) -> Eigen::MatrixXd {
        Eigen::VectorXd const& position = state.position;

        Eigen::MatrixXd dynMatrix = Eigen::MatrixXd::Zero(state.size(), state.size());
        dynMatrix.block(0, position.size(), position.size(), position.size()) =
            Eigen::MatrixXd::Identity(position.size(), position.size());

        return dynMatrix;
    };
}

/*! Write the output data to appropriate messages given the state components
 @return void
 @param uint64_t currentSimNanos
 */
void LinearODeKF::writeOutputMessages(uint64_t currentSimNanos) {
    NavTransMsgPayload navTransOutMsgBuffer{};
    FilterMsgPayload opNavFilterMsgBuffer{};
    FilterResidualsMsgPayload residualsBuffer{};

    /*! - Write the flyby OD estimate into the copy of the navigation message structure*/
    eigenMatrixXToCArray(this->ekf.stateLogged.scale(1 / this->ekf.unitConversion).position,
                         navTransOutMsgBuffer.r_BN_N);
    eigenMatrixXToCArray(this->ekf.stateLogged.scale(1 / this->ekf.unitConversion).velocity,
                         navTransOutMsgBuffer.v_BN_N);

    /*! - Populate the filter states output buffer and write the output message*/
    opNavFilterMsgBuffer.timeTag = (double)this->previousSimNanos * NANO2SEC;
    eigenVectorToCArray(this->ekf.stateLogged.scale(1 / this->ekf.unitConversion).position, opNavFilterMsgBuffer.state + 0);
    eigenVectorToCArray(this->ekf.stateLogged.scale(1 / this->ekf.unitConversion).velocity, opNavFilterMsgBuffer.state + 3);
    eigenMatrixXToCArray(1 / this->ekf.unitConversion * this->ekf.stateError, opNavFilterMsgBuffer.stateError);
    eigenMatrixXToCArray(1 / this->ekf.unitConversion / this->ekf.unitConversion * this->ekf.covar, opNavFilterMsgBuffer.covar);
    opNavFilterMsgBuffer.numberOfStates = this->ekf.state.size();

    for (
        auto it = this->measurements.popEarliest();
        it.has_value();
        it = this->measurements.popEarliest()
    ) {
        auto& measurement = it.value().second;

        residualsBuffer.valid = true;
        residualsBuffer.numberOfObservations = 1;
        residualsBuffer.sizeOfObservations = measurement.observation.size();
        eigenMatrixXToCArray(measurement.observation, residualsBuffer.observation);
        eigenMatrixXToCArray(measurement.postFitResiduals, residualsBuffer.postFits);
        eigenMatrixXToCArray(measurement.preFitResiduals, residualsBuffer.preFits);
    }

    this->opNavResidualMsg.write(&residualsBuffer, this->moduleID, currentSimNanos);
    this->navTransOutMsg.write(&navTransOutMsgBuffer, this->moduleID, currentSimNanos);
    this->opNavFilterMsg.write(&opNavFilterMsgBuffer, this->moduleID, currentSimNanos);
}

/*! Read the message containing the measurement data.
 * It updates class variables relating to measurement data including validity and time tags.
 @return void
 */
void LinearODeKF::readFilterMeasurements() {
    this->opNavHeadingBuffer = this->opNavHeadingMsg();

    if (!this->opNavHeadingBuffer.valid) return;
    if (this->opNavHeadingBuffer.timeTag < (double)this->previousSimNanos * NANO2SEC) return;

    /*! - Read measurement and cholesky decomposition its noise*/
    LinearODeKFMeasurementModel headingMeasurement = {};
    headingMeasurement.observation = cArrayToEigenVector(this->opNavHeadingBuffer.rhat_BN_N).normalized();
    headingMeasurement.measNoise =
        ( this->measNoiseScaling
        * cArrayToEigenMatrixX(
            this->opNavHeadingBuffer.covar_N,
            (int)headingMeasurement.observation.size(),
            (int)headingMeasurement.observation.size() )
        );

    this->measurements.enqueue(this->opNavHeadingBuffer.timeTag, std::move(headingMeasurement));
}

/*! Set the filter measurement noise scale factor if desirable
    @param double measurementNoiseScale
    @return void
    */
void LinearODeKF::setMeasurementNoiseScale(const double measurementNoiseScale) {
    this->measNoiseScaling = measurementNoiseScale;
}

/*! Get the filter measurement noise scale factor
    @return double measurementNoiseScale
    */
double LinearODeKF::getMeasurementNoiseScale() const { return this->measNoiseScaling; }
