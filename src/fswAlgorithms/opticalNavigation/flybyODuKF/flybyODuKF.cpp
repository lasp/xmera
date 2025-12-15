#include "flybyODuKF.h"

/*! Reset the flyby OD filter to an initial state and
 initializes the internal estimation matrices.
 @return void
 @param currentSimNanos The clock time at which the function was called (nanoseconds)
 */
void FlybyODuKF::customreset() {
    /*! - Check if the required message has not been connected */
    assert(this->opNavHeadingMsg.isLinked());
    /*! - Initialize filter parameters and change units to km and s */
    this->muCentral *= pow(this->unitConversion, 3);  // mu is input in meters
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
    this->dynamics.setDynamics(twoBodyDynamics);
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
    eigenMatrixXToCArray(this->state.scale(1 / this->unitConversion).getPositionStates(), navTransOutMsgBuffer.r_BN_N);
    eigenMatrixXToCArray(this->state.scale(1 / this->unitConversion).getVelocityStates(), navTransOutMsgBuffer.v_BN_N);

    /*! - Populate the filter states output buffer and write the output message*/
    opNavFilterMsgBuffer.timeTag = this->previousFilterTimeTag;
    eigenMatrixXToCArray(this->state.scale(1 / this->unitConversion).returnValues(), opNavFilterMsgBuffer.state);
    eigenMatrixXToCArray(this->xBar.scale(1 / this->unitConversion).returnValues(), opNavFilterMsgBuffer.stateError);
    eigenMatrixXToCArray(1 / this->unitConversion / this->unitConversion * this->covar, opNavFilterMsgBuffer.covar);
    opNavFilterMsgBuffer.numberOfStates = this->state.size();

    auto optionalMeasurement = this->measurements[0];
    if (optionalMeasurement.has_value()) {
        auto measurement = MeasurementModel();
        measurement = optionalMeasurement.value();
        residualsBuffer.valid = true;
        residualsBuffer.numberOfObservations = 1;
        residualsBuffer.sizeOfObservations = measurement.getObservation().size();
        eigenMatrixXToCArray(measurement.getObservation(), residualsBuffer.observation);
        eigenMatrixXToCArray(measurement.getPostFitResiduals(), residualsBuffer.postFits);
        eigenMatrixXToCArray(measurement.getPreFitResiduals(), residualsBuffer.preFits);
        this->measurements[0].reset();
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

    if (headingMeasurement.getValidity() && headingMeasurement.getTimeTag() >= this->previousFilterTimeTag) {
        /*! - Read measurement and cholesky decomposition its noise*/
        headingMeasurement.setObservation(cArrayToEigenVector(this->opNavHeadingBuffer.rhat_BN_N));
        headingMeasurement.getObservation().normalize();
        headingMeasurement.setMeasurementNoise(this->measNoiseScaling *
                                               cArrayToEigenMatrixX(this->opNavHeadingBuffer.covar_N,
                                                                    (int)headingMeasurement.size(),
                                                                    (int)headingMeasurement.size()));
        headingMeasurement.setMeasurementModel(MeasurementModel::normalizedPositionStates);
        this->measurements[0] = headingMeasurement;
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
