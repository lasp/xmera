// SPDX-License-Identifier: ISC
// Copyright (c) 2023, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

#include "flybyODuKF.h"

void FlybyODuKF::reset(uint64_t currentSimNanos) {
    this->customReset();
    this->srukf.reset();
    this->previousSimNanos = currentSimNanos;
}

void FlybyODuKF::updateState(uint64_t currentSimNanos) {
    this->readFilterMeasurements();
    xmera::updateKalmanFilter(
        this->srukf,
        std::span{&this->measurement, 1},
        (double)this->previousSimNanos * NANO2SEC,
        (double)currentSimNanos * NANO2SEC
    );
    this->writeOutputMessages(currentSimNanos);

    this->previousSimNanos = currentSimNanos;
}

/*! Reset the flyby OD filter to an initial state and
 initializes the internal estimation matrices.
 @return void
 @param currentSimNanos The clock time at which the function was called (nanoseconds)
 */
void FlybyODuKF::customReset() {
    /*! - Check if the required message has not been connected */
    assert(this->opNavHeadingMsg.isLinked());
    /*! - Initialize filter parameters and change units to km and s */
    this->muCentral *= pow(this->srukf.unitConversion, 3);  // mu is input in meters
    double centralBody = this->muCentral;
    std::function<FilterStateVector(double, const FilterStateVector)> twoBodyDynamics =
        [centralBody](double t, const FilterStateVector& state) {
            FilterStateVector XDot;
            /*! Implement propagation with rate derivatives set to zero */
            /*! Implement point mass gravity for the propagation */
            PositionState flybyPosition;
            VelocityState flybyVelocity;
            flybyPosition.setValues(state.getVelocityStates());
            flybyVelocity.setValues(-centralBody / pow(state.getPositionStates().norm(), 3) *
                                    state.getPositionStates());

            XDot.setPosition(flybyPosition);
            XDot.setVelocity(flybyVelocity);

            return XDot;
        };

    /*! - Set the filter dynamics */
    this->srukf.dynamics.setDynamics(twoBodyDynamics);
}

/*! Read the message containing the measurement data.
 * It updates class variables relating to measurement data including validity and time tags.
 @return void
 */
void FlybyODuKF::writeOutputMessages(uint64_t currentSimNanos) {
    NavTransMsgPayload navTransOutMsgBuffer{};
    FilterMsgPayload opNavFilterMsgBuffer{};
    FilterResidualsMsgPayload residualsBuffer{};

    /*! - Write the flyby OD estimate into the copy of the navigation message structure*/
    eigenMatrixXToCArray(this->srukf.state.scale(1 / this->srukf.unitConversion).getPositionStates(), navTransOutMsgBuffer.r_BN_N);
    eigenMatrixXToCArray(this->srukf.state.scale(1 / this->srukf.unitConversion).getVelocityStates(), navTransOutMsgBuffer.v_BN_N);

    /*! - Populate the filter states output buffer and write the output message*/
    opNavFilterMsgBuffer.timeTag = (double)this->previousSimNanos * NANO2SEC;
    eigenMatrixXToCArray(this->srukf.state.scale(1 / this->srukf.unitConversion).returnValues(), opNavFilterMsgBuffer.state);
    eigenMatrixXToCArray(this->srukf.xBar.scale(1 / this->srukf.unitConversion).returnValues(), opNavFilterMsgBuffer.stateError);
    eigenMatrixXToCArray(1 / this->srukf.unitConversion / this->srukf.unitConversion * this->srukf.covar, opNavFilterMsgBuffer.covar);
    opNavFilterMsgBuffer.numberOfStates = this->srukf.state.size();

    if (this->measurement.has_value()) {
        auto measurement = std::exchange(this->measurement, std::nullopt).value();

        residualsBuffer.valid = true;
        residualsBuffer.numberOfObservations = 1;
        residualsBuffer.sizeOfObservations = measurement.getObservation().size();
        eigenMatrixXToCArray(measurement.getObservation(), residualsBuffer.observation);
        eigenMatrixXToCArray(measurement.getPostFitResiduals(), residualsBuffer.postFits);
        eigenMatrixXToCArray(measurement.getPreFitResiduals(), residualsBuffer.preFits);
    }
    this->opNavResidualMsg.write(&residualsBuffer, this->moduleID, currentSimNanos);
    this->navTransOutMsg.write(&navTransOutMsgBuffer, this->moduleID, currentSimNanos);
    this->opNavFilterMsg.write(&opNavFilterMsgBuffer, this->moduleID, currentSimNanos);
}

/*! Read the message containing the measurement data.
 * It updates class variables relating to measurement data including validity and time tags.
 @return void
 */
void FlybyODuKF::readFilterMeasurements() {
    this->opNavHeadingBuffer = this->opNavHeadingMsg();
    auto headingMeasurement = MeasurementModel();

    headingMeasurement.setTimeTag(this->opNavHeadingBuffer.timeTag);
    headingMeasurement.setValidity(this->opNavHeadingBuffer.valid);

    if (headingMeasurement.getValidity() && headingMeasurement.getTimeTag() >= (double)this->previousSimNanos * NANO2SEC) {
        /*! - Read measurement and cholesky decomposition its noise*/
        headingMeasurement.setObservation(cArrayToEigenVector(this->opNavHeadingBuffer.rhat_BN_N));
        headingMeasurement.getObservation().normalize();
        headingMeasurement.setMeasurementNoise(this->srukf.getMeasurementNoiseScale() *
                                               cArrayToEigenMatrixX(this->opNavHeadingBuffer.covar_N,
                                                                    (int)headingMeasurement.size(),
                                                                    (int)headingMeasurement.size()));
        headingMeasurement.setMeasurementModel(MeasurementModel::normalizedPositionStates);
        this->measurement = headingMeasurement;
    }
}

/*! Set the gravitational parameter used for orbit propagation
    @param double muInput
    @return void
    */
void FlybyODuKF::setCentralBodyGravitationParameter(const double muInput) { this->muCentral = muInput; }

/*! Get gravitational parameter used for orbit propagation
    @return double muCentral
    */
double FlybyODuKF::getCentralBodyGravitationParameter() const { return this->muCentral; }
