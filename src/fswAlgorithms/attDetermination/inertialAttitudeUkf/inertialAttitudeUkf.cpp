// SPDX-License-Identifier: ISC
// Copyright (c) 2024, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

#include "inertialAttitudeUkf.h"

#include <span>

InertialAttitudeUkf::InertialAttitudeUkf(const AttitudeFilterMethod method) { this->measurementAcceptanceMethod = method; }

void InertialAttitudeUkf::reset(uint64_t currentSimNanos) {
    this->customReset();
    this->previousSimNanos = currentSimNanos;
}

void InertialAttitudeUkf::updateState(uint64_t currentSimNanos) {
    this->customInitializeUpdate();
    this->readFilterMeasurements();

    this->measurements.applyToFilter(
        this->srukf,
        (double)this->previousSimNanos * NANO2SEC,
        (double)currentSimNanos * NANO2SEC
    );
    this->previousSimNanos = currentSimNanos;

    this->customFinalizeUpdate();
    this->writeOutputMessages(currentSimNanos);
}

void InertialAttitudeUkf::customReset() {
    /*! No custom reset for this module */
    std::function<FilterStateVector(double, const FilterStateVector)> attitudeDynamics =
        [this](double t, const FilterStateVector& state) {
            Eigen::Vector3d mrp(state.getPositionStates());
            Eigen::Vector3d omega(state.getVelocityStates());
            Eigen::MatrixXd bMat = bmatMrp(mrp);

            FilterStateVector stateDerivative;
            PositionState mrpDot;
            mrpDot.setValues(0.25 * bMat * omega);
            stateDerivative.setPosition(mrpDot);

            Eigen::Vector3d wheelTorque = Eigen::Vector3d::Zero();
            for (int i = 0; i < this->rwArrayConfigPayload.numRW; i++) {
                Eigen::Vector3d gsMatrix = Eigen::Map<Eigen::Vector3d>(&this->rwArrayConfigPayload.GsMatrix_B[i * 3]);
                wheelTorque += this->wheelAccelerations[i] * this->rwArrayConfigPayload.JsList[i] * gsMatrix;
            }

            VelocityState omegaDot;
            omegaDot.setValues(-this->spacecraftInertiaInverse *
                               (tildeMatrix(omega) * this->spacecraftInertia * omega + wheelTorque));
            stateDerivative.setVelocity(omegaDot);

            if (state.hasBias()) {
                BiasState xDotBias;
                xDotBias.setValues(Eigen::VectorXd::Zero(3));
                stateDerivative.setBias(xDotBias);
            }

            return stateDerivative;
        };
    this->srukf.dynamics.setDynamics(attitudeDynamics);

    this->firstFilterPass = true;
}

/*! Before every update, check the MRP norm for a shadow set switch
 @return void
 */
void InertialAttitudeUkf::customInitializeUpdate() { this->switchStateCovariance(); }

/*! After every update, check the MRP norm for a shadow set switch
 @return void
 */
void InertialAttitudeUkf::customFinalizeUpdate() { this->switchStateCovariance(); }

/*! Check the norm of the mrp and switch both the position state of the state vector (the mrp) and the covariance
 * if above the desired threshold
 @return void
 */
void InertialAttitudeUkf::switchStateCovariance() {
    Eigen::Vector3d sigma = this->srukf.state.getPositionStates();
    if (sigma.norm() > this->mrpSwitchThreshold) {
        PositionState mrp;
        mrp.setValues(mrpSwitch(sigma, this->mrpSwitchThreshold));
        this->srukf.state.setPosition(mrp);
        Eigen::Matrix3d switchMatrix = 2 * std::pow(sigma.norm(), 4) * sigma * sigma.transpose() -
                                       std::pow(sigma.norm(), 2) * Eigen::Matrix3d::Identity();
        this->srukf.covar.block(0, 0, 3, 3) = switchMatrix * this->srukf.covar.block(0, 0, 3, 3) * switchMatrix.transpose();
        this->srukf.covar.block(0, 3, 3, 3) = switchMatrix * this->srukf.covar.block(0, 3, 3, 3);
        this->srukf.covar.block(3, 0, 3, 3) = this->srukf.covar.block(3, 0, 3, 3) * switchMatrix.transpose();
    }
}

/*! Read the message containing the measurement data.
 * It updates class variables relating to measurement data including validity and time tags.
 @return void
 */
void InertialAttitudeUkf::writeOutputMessages(uint64_t currentSimNanos) {
    NavAttMsgPayload navAttPayload{};
    FilterMsgPayload filterPayload{};
    FilterResidualsMsgPayload attitudePayload{};
    FilterResidualsMsgPayload ratePayload{};

    /*! - Write the flyby OD estimate into the copy of the navigation message structure*/
    navAttPayload.timeTag = (double)this->previousSimNanos * NANO2SEC;
    eigenMatrixXToCArray(this->srukf.state.getPositionStates(), navAttPayload.sigma_BN);
    eigenMatrixXToCArray(this->srukf.state.getVelocityStates(), navAttPayload.omega_BN_B);

    /*! - Populate the filter states output buffer and write the output message*/
    filterPayload.timeTag = (double)this->previousSimNanos * NANO2SEC;
    eigenMatrixXToCArray(this->srukf.state.returnValues(), filterPayload.state);
    eigenMatrixXToCArray(this->srukf.xBar.returnValues(), filterPayload.stateError);
    eigenMatrixXToCArray(this->srukf.covar, filterPayload.covar);

    for (
        auto it = this->measurements.popEarliest();
        it.has_value();
        it = this->measurements.popEarliest()
    ) {
        auto& measurement = it.value().second;

        switch (measurement.type) {
        case InertialAttitudeUkfMeasurementType::Attitude:
            attitudePayload.valid = true;
            attitudePayload.numberOfObservations = 1;
            attitudePayload.sizeOfObservations = measurement.observation.size();
            eigenMatrixXToCArray(measurement.observation, attitudePayload.observation);
            eigenMatrixXToCArray(measurement.postFitResiduals, attitudePayload.postFits);
            eigenMatrixXToCArray(measurement.preFitResiduals, attitudePayload.preFits);
            break;

        case InertialAttitudeUkfMeasurementType::Rate:
            ratePayload.valid = true;
            ratePayload.numberOfObservations = 1;
            ratePayload.sizeOfObservations = measurement.observation.size();
            eigenMatrixXToCArray(measurement.observation, ratePayload.observation);
            eigenMatrixXToCArray(measurement.postFitResiduals, ratePayload.postFits);
            eigenMatrixXToCArray(measurement.preFitResiduals, ratePayload.preFits);
        }
    }

    this->attitudeResidualMsg.write(&attitudePayload, this->moduleID, currentSimNanos);
    this->rateResidualMsg.write(&ratePayload, this->moduleID, currentSimNanos);

    this->navAttitudeOutputMsg.write(&navAttPayload, this->moduleID, currentSimNanos);
    this->inertialFilterOutputMsg.write(&filterPayload, this->moduleID, currentSimNanos);
}

/*! Read current RW speends and populate the accelerations in order to propagate
 * @return void
 * */
void InertialAttitudeUkf::readRWSpeedData() {
    RWSpeedMsgPayload rwSpeedPayload = this->rwSpeedMsg();
    uint64_t wheelSpeedTime = this->rwSpeedMsg.timeWritten();
    if (this->firstFilterPass) {
        this->wheelAccelerations = Eigen::VectorXd::Zero(this->rwArrayConfigPayload.numRW);
        Eigen::MatrixXd wheelSpeed =
            cArrayToEigenMatrixX(rwSpeedPayload.wheelSpeeds, this->rwArrayConfigPayload.numRW, 1);
        this->previousWheelSpeeds = Eigen::Map<Eigen::VectorXd>(wheelSpeed.data(), wheelSpeed.size());
        this->previousWheelSpeedTime = wheelSpeedTime * NANO2SEC;
    } else {
        double dt = wheelSpeedTime * NANO2SEC - this->previousWheelSpeedTime;
        Eigen::MatrixXd wheelSpeed =
            cArrayToEigenMatrixX(rwSpeedPayload.wheelSpeeds, this->rwArrayConfigPayload.numRW, 1);
        Eigen::VectorXd currentWheelSpeed = Eigen::Map<Eigen::VectorXd>(wheelSpeed.data(), wheelSpeed.size());
        this->wheelAccelerations = (currentWheelSpeed - this->previousWheelSpeeds) / dt;
        this->previousWheelSpeeds = Eigen::Map<Eigen::VectorXd>(wheelSpeed.data(), wheelSpeed.size());
        this->previousWheelSpeedTime = wheelSpeedTime * NANO2SEC;
    }
}

/*! Loop through the all the input attitude data
 * @return void
 * */
void InertialAttitudeUkf::readAttitudeData() {
    auto actualAttitudeMessages = std::span{this->attitudeMessages.data(), this->numberOfStarTackers};

    for (auto& attitudeMessage : actualAttitudeMessages) {
        auto attitude = attitudeMessage.attitudeMsg();

        this->validAttitude = (attitude.timeTag > (double)this->previousSimNanos * NANO2SEC);
        if (!this->validAttitude) continue;

        /*! - Only consider the filter started once a Star Tracker image is processed */
        this->firstFilterPass = false;

        /*! - Get the mapping from camera frame to inertial for the noise matrix */
        Eigen::Matrix3d dcm_CB = cArrayToEigenMatrix3(attitude.dcm_CB);

        InertialAttitudeUkfMeasurementModel attitudeMeasurement = {};
        attitudeMeasurement.type = InertialAttitudeUkfMeasurementType::Attitude;
        attitudeMeasurement.observation = mrpSwitch(
            Eigen::Map<Eigen::Vector3d>(attitude.MRP_BdyInrtl).eval(),
            this->mrpSwitchThreshold
        );
        attitudeMeasurement.measNoise =
            ( this->measNoiseScaling
            * dcm_CB.transpose()
            * attitudeMessage.measurementNoise_C
            * dcm_CB
            );

        this->measurements.enqueue(attitude.timeTag, std::move(attitudeMeasurement));
    }
}

/*! Read the message containing the rate data to be processed my the filter as a measurement
 * @return void
 * */
void InertialAttitudeUkf::readRateData() {
    STAttMsgPayload rateBuffer = this->rateDataInMsg();

    if (rateBuffer.timeTag <= (double)this->previousSimNanos * NANO2SEC) return;

    InertialAttitudeUkfMeasurementModel rateMeasurement = {};
    rateMeasurement.type = InertialAttitudeUkfMeasurementType::Rate;
    rateMeasurement.observation = cArrayToEigenVector(rateBuffer.omega_BN_B);
    rateMeasurement.measNoise =
        ( this->measNoiseScaling
        * this->rateNoise
        );

    this->measurements.enqueue(rateBuffer.timeTag, std::move(rateMeasurement));
}

/*! Read the message containing the measurement data.
 * It updates class variables relating to measurement data including validity and time tags.
 @return void
 */
void InertialAttitudeUkf::readFilterMeasurements() {
    /*! - Read static RW and spacecraft config data message and store it in module fields */
    if (this->firstFilterPass) {
        this->rwArrayConfigPayload = this->rwArrayConfigMsg();
        auto vehicleConfigPayload = this->vehicleConfigMsg();

        this->spacecraftInertia = cArrayToEigenMatrix3(vehicleConfigPayload.ISCPntB_B);
        this->spacecraftInertiaInverse = this->spacecraftInertia.inverse();
    }

    this->measurementIndex = 0;
    /*! - Read in wheel speeds, their time, and compute the wheel accelerations for the propagation method*/
    readRWSpeedData();
    /*! - Read star tracker measurements*/
    readAttitudeData();
    /*! Only add the rate measurements to processing if the filter is in a mode that desires that */
    if (this->measurementAcceptanceMethod == AttitudeFilterMethod::AllMeasurements) {
        readRateData();
    }
    if (measurementAcceptanceMethod == AttitudeFilterMethod::RateMeasurementsWhenNoStars && !this->validAttitude) {
        readRateData();
    }
}

/*! Set the rate measurement noise matrix
    @param Eigen::Matrix3d rateNoise
    @return void
    */
void InertialAttitudeUkf::setRateNoise(const Eigen::Matrix3d& rateNoiseInput) { this->rateNoise = rateNoiseInput; }

/*! Get the rate measurement noise matrix
    @return Eigen::Matrix3d rateNoise
    */
Eigen::Matrix3d InertialAttitudeUkf::getRateNoise() const { return this->rateNoise; }

/*! Add a star tracker to the filter solution using the attitudeMessage class
    @return attitudeMessage attitude
    */
void InertialAttitudeUkf::addAttitudeInput(const AttitudeMessage& attitudeMsg) {
    this->attitudeMessages[this->numberOfStarTackers] = attitudeMsg;
    this->numberOfStarTackers += 1;
}

/*! Get the star tracker measurement noise matrix for a particular number (indexed at 0)
    @param int attitudeMeasNumber
    @return Eigen::Matrix3d attitudeMeasNoise
    */
Eigen::Matrix3d InertialAttitudeUkf::getAttitudeNoise(int attitudeMeasNumber) const {
    assert(attitudeMeasNumber < this->numberOfStarTackers);
    return this->attitudeMessages[attitudeMeasNumber].measurementNoise_C;
}

/*! Set the filter measurement noise scale factor if desirable
    @param double measurementNoiseScale
    @return void
    */
void InertialAttitudeUkf::setMeasurementNoiseScale(const double measurementNoiseScale) {
    this->measNoiseScaling = measurementNoiseScale;
}

/*! Get the filter measurement noise scale factor
    @return double measurementNoiseScale
    */
double InertialAttitudeUkf::getMeasurementNoiseScale() const { return this->measNoiseScaling; }
