// SPDX-License-Identifier: ISC
// Copyright (c) 2024, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

#include "linearODeKF.h"

void LinearODeKF::reset(uint64_t currentSimNanos) {
    this->customReset();
    this->ekf.reset(currentSimNanos);
}

void LinearODeKF::updateState(uint64_t currentSimNanos) {
    this->readFilterMeasurements();
    this->ekf.updateState(currentSimNanos, this->measurements);
    this->writeOutputMessages(currentSimNanos);
}

/*! Reset the flyby OD filter to an initial state and initializes the internal estimation matrices.
 @return void
 @param currentSimNanos The clock time at which the function was called (nanoseconds)
 */
void LinearODeKF::customReset() {
    /*! - Check if the required message has not been connected */
    assert(this->opNavHeadingMsg.isLinked());
    if (this->constantVelocityInitial) {
        this->constantVelocity = this->constantVelocityInitial.value() * this->ekf.unitConversion;
    }
    /*! - Set the dynamics matrix calculator*/
    std::function<Eigen::MatrixXd(double, const FilterStateVector)> dynamicsMatrixCalculator =
        [this](double time, const FilterStateVector& state) {
            Eigen::VectorXd position = state.getPositionStates();
            Eigen::MatrixXd dynamicsMatrix = Eigen::MatrixXd::Zero(state.size(), state.size());
            if (!this->constantVelocity) {
                dynamicsMatrix.block(0, position.size(), position.size(), position.size()) =
                    Eigen::MatrixXd::Identity(position.size(), position.size());
            }
            return dynamicsMatrix;
        };

    this->ekf.dynamics.setDynamicsMatrix(dynamicsMatrixCalculator);

    /*! - Set the filter dynamics (linear) */
    std::function<FilterStateVector(double, const FilterStateVector)> twoBodyDynamics =
        [this](double t, const FilterStateVector& state) {
            FilterStateVector XDot;
            PositionState stateDerivative;

            if (this->constantVelocity) {
                stateDerivative.setValues(this->constantVelocity.value());
            } else {
                stateDerivative.setValues(state.getVelocityStates());
                VelocityState flybyVelocity;
                flybyVelocity.setValues(Eigen::Vector3d::Zero());
                XDot.setVelocity(flybyVelocity);
            }

            XDot.setPosition(stateDerivative);
            Eigen::MatrixXd stm = state.detachStm();

            Eigen::MatrixXd dynMatrix = this->ekf.dynamics.computeDynamicsMatrix(t, state);
            XDot.attachStm(dynMatrix * stm);

            return XDot;
        };
    this->ekf.dynamics.setDynamics(twoBodyDynamics);
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
    eigenMatrixXToCArray(this->ekf.stateLogged.scale(1 / this->ekf.unitConversion).getPositionStates(),
                         navTransOutMsgBuffer.r_BN_N);
    if (this->constantVelocityInitial) {
        eigenMatrixToCArray(constantVelocityInitial.value(), navTransOutMsgBuffer.v_BN_N);
    } else {
        eigenMatrixXToCArray(this->ekf.stateLogged.scale(1 / this->ekf.unitConversion).getVelocityStates(),
                             navTransOutMsgBuffer.v_BN_N);
    }

    /*! - Populate the filter states output buffer and write the output message*/
    opNavFilterMsgBuffer.timeTag = this->ekf.previousFilterTimeTag;
    eigenMatrixXToCArray(this->ekf.stateLogged.scale(1 / this->ekf.unitConversion).returnValues(), opNavFilterMsgBuffer.state);
    eigenMatrixXToCArray(1 / this->ekf.unitConversion * this->ekf.stateError, opNavFilterMsgBuffer.stateError);
    eigenMatrixXToCArray(1 / this->ekf.unitConversion / this->ekf.unitConversion * this->ekf.covar, opNavFilterMsgBuffer.covar);
    opNavFilterMsgBuffer.numberOfStates = this->ekf.state.size();

    auto optionalMeasurement = this->measurements[0];
    if (optionalMeasurement.has_value()) {
        auto measurement = MeasurementModel();
        measurement = optionalMeasurement.value();
        residualsBuffer.valid = true;
        residualsBuffer.numberOfObservations = 1;
        residualsBuffer.sizeOfObservations = measurement.size();
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
void LinearODeKF::readFilterMeasurements() {
    this->opNavHeadingBuffer = this->opNavHeadingMsg();
    auto headingMeasurement = MeasurementModel();

    headingMeasurement.setTimeTag(this->opNavHeadingBuffer.timeTag);
    headingMeasurement.setValidity(this->opNavHeadingBuffer.valid);

    if (headingMeasurement.getValidity() && headingMeasurement.getTimeTag() >= this->ekf.previousFilterTimeTag) {
        /*! - Read measurement and cholesky decomposition its noise*/
        headingMeasurement.setObservation(cArrayToEigenVector(this->opNavHeadingBuffer.rhat_BN_N).normalized());
        headingMeasurement.setMeasurementNoise(this->ekf.measNoiseScaling *
                                               cArrayToEigenMatrixX(this->opNavHeadingBuffer.covar_N,
                                                                    (int)headingMeasurement.size(),
                                                                    (int)headingMeasurement.size()));
        headingMeasurement.setMeasurementModel(MeasurementModel::normalizedPositionStates);
        headingMeasurement.setMeasurementMatrix(LinearODeKF::measurementMatrix);
        this->measurements[0] = headingMeasurement;
    }
}

/*! Compute the measurement matrix to linearize the measurement model
    @param FilterStateVector state
    @return Eigen::MatrixXd
*/
Eigen::MatrixXd LinearODeKF::measurementMatrix(const FilterStateVector& state) {
    Eigen::Vector3d position = state.getPositionStates();
    Eigen::MatrixXd measurementMatrix = Eigen::MatrixXd::Zero(position.size(), state.size());
    measurementMatrix.block(0, 0, position.size(), position.size()) =
        1 / position.norm() *
        (Eigen::MatrixXd::Identity(position.size(), position.size()) -
         1 / pow(position.norm(), 2) * position * position.transpose());

    return measurementMatrix;
}

/*! Set a constant velocity vector that will not be estimated. Assert that velocity is not part of the state
    @param Eigen::Vector3d velocity
    */
void LinearODeKF::setConstantVelocity(const Eigen::Vector3d& velocity) {
    assert(this->ekf.state.hasVelocity() == false);
    this->constantVelocityInitial = velocity;
}

/*! Get the constant velocity vector
    @return std::optional<Eigen::Vector3d> velocity
    */
std::optional<Eigen::Vector3d> LinearODeKF::getConstantVelocity() const { return this->constantVelocityInitial; }
