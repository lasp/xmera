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

    this->measurements.applyToFilter(
        this->srukf,
        (double)this->previousSimNanos * NANO2SEC,
        (double)currentSimNanos * NANO2SEC
    );
    this->previousSimNanos = currentSimNanos;

    this->writeOutputMessages(currentSimNanos);
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

    /*! - Set the filter dynamics */
    this->srukf.dynamics = [centralBody](double t, const FilterStateVector& state) {
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
void FlybyODuKF::readFilterMeasurements() {
    this->opNavHeadingBuffer = this->opNavHeadingMsg();

    if (!this->opNavHeadingBuffer.valid) return;
    if (this->opNavHeadingBuffer.timeTag < (double)this->previousSimNanos * NANO2SEC) return;

    /*! - Read measurement and cholesky decomposition its noise*/
    auto observation = cArrayToEigenVector(this->opNavHeadingBuffer.rhat_BN_N).normalized();

    auto headingMeasurement = FlybyODuKFMeasurementModel();
    headingMeasurement.observation = observation;
    headingMeasurement.measNoise =
        ( this->measNoiseScaling
        * cArrayToEigenMatrixX(
            this->opNavHeadingBuffer.covar_N,
            (int)observation.size(),
            (int)observation.size()
          )
        );

    this->measurements.enqueue(this->opNavHeadingBuffer.timeTag, std::move(headingMeasurement));
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

/*! Set the filter measurement noise scale factor if desirable
    @param double measurementNoiseScale
    @return void
    */
void FlybyODuKF::setMeasurementNoiseScale(const double measurementNoiseScale) {
    this->measNoiseScaling = measurementNoiseScale;
}

/*! Get the filter measurement noise scale factor
    @return double measurementNoiseScale
    */
double FlybyODuKF::getMeasurementNoiseScale() const { return this->measNoiseScaling; }
