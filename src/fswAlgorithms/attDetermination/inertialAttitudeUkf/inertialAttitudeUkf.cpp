// SPDX-License-Identifier: ISC
// Copyright (c) 2024, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

#include "inertialAttitudeUkf.h"

InertialAttitudeUkf::InertialAttitudeUkf(AttitudeFilterMethod method) { this->measurementAcceptanceMethod = method; }

void InertialAttitudeUkf::customreset() {
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
    this->dynamics.setDynamics(attitudeDynamics);

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
    Eigen::Vector3d sigma = this->state.getPositionStates();
    if (sigma.norm() > this->mrpSwitchThreshold) {
        PositionState mrp;
        mrp.setValues(mrpSwitch(sigma, this->mrpSwitchThreshold));
        this->state.setPosition(mrp);
        Eigen::Matrix3d switchMatrix = 2 * std::pow(sigma.norm(), 4) * sigma * sigma.transpose() -
                                       std::pow(sigma.norm(), 2) * Eigen::Matrix3d::Identity();
        this->covar.block(0, 0, 3, 3) = switchMatrix * this->covar.block(0, 0, 3, 3) * switchMatrix.transpose();
        this->covar.block(0, 3, 3, 3) = switchMatrix * this->covar.block(0, 3, 3, 3);
        this->covar.block(3, 0, 3, 3) = this->covar.block(3, 0, 3, 3) * switchMatrix.transpose();
    }
}

/*! Read the message containing the measurement data.
 * It updates class variables relating to measurement data including validity and time tags.
 @return void
 */
void InertialAttitudeUkf::writeOutputMessages(uint64_t currentSimNanos) {
    NavAttMsgPayload navAttPayload{};
    FilterMsgPayload filterPayload{};
    FilterResidualsMsgPayload starTrackerPayload{};
    FilterResidualsMsgPayload gyroPayload{};

    /*! - Write the flyby OD estimate into the copy of the navigation message structure*/
    navAttPayload.timeTag = this->previousFilterTimeTag;
    eigenMatrixXToCArray(this->state.getPositionStates(), navAttPayload.sigma_BN);
    eigenMatrixXToCArray(this->state.getVelocityStates(), navAttPayload.omega_BN_B);

    /*! - Populate the filter states output buffer and write the output message*/
    filterPayload.timeTag = this->previousFilterTimeTag;
    eigenMatrixXToCArray(this->state.returnValues(), filterPayload.state);
    eigenMatrixXToCArray(this->xBar.returnValues(), filterPayload.stateError);
    eigenMatrixXToCArray(this->covar, filterPayload.covar);

    for (size_t index = 0; index < MAX_MEASUREMENT_NUMBER; index++) {
        if (this->measurements[index].has_value()) {
            auto measurement = this->measurements[index].value();
            if (measurement.getMeasurementName() == "starTracker") {
                starTrackerPayload.valid = true;
                starTrackerPayload.numberOfObservations = 1;
                starTrackerPayload.sizeOfObservations = measurement.size();
                eigenMatrixXToCArray(measurement.getObservation(), starTrackerPayload.observation);
                eigenMatrixXToCArray(measurement.getPostFitResiduals(), starTrackerPayload.postFits);
                eigenMatrixXToCArray(measurement.getPreFitResiduals(), starTrackerPayload.preFits);
            }
            if (measurement.getMeasurementName() == "gyro") {
                gyroPayload.valid = true;
                gyroPayload.numberOfObservations = 1;
                gyroPayload.sizeOfObservations = measurement.size();
                eigenMatrixXToCArray(measurement.getObservation(), gyroPayload.observation);
                eigenMatrixXToCArray(measurement.getPostFitResiduals(), gyroPayload.postFits);
                eigenMatrixXToCArray(measurement.getPreFitResiduals(), gyroPayload.preFits);
            }
            this->measurements[index].reset();
        }
    }
    this->starTrackerResidualMsg.write(&starTrackerPayload, this->moduleID, currentSimNanos);
    this->gyroResidualMsg.write(&gyroPayload, this->moduleID, currentSimNanos);

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

/*! Loop through the all the input star trackers and populate their measurement container if they are foward
 * in time
 * @return void
 * */
void InertialAttitudeUkf::readStarTrackerData() {
    for (int index = 0; index < this->numberOfStarTackers; index++) {
        auto starTracker = this->starTrackerMessages[index].starTrackerMsg();
        if (starTracker.timeTag > this->previousFilterTimeTag) {
            auto starTrackerMeasurement = MeasurementModel();
            starTrackerMeasurement.setMeasurementName("starTracker");
            starTrackerMeasurement.setTimeTag(starTracker.timeTag);
            starTrackerMeasurement.setValidity(true);

            /*! - Get the mapping from camera frame to inertial for the noise matrix */
            Eigen::Matrix3d dcm_CB = cArrayToEigenMatrix3(starTracker.dcm_CB);

            starTrackerMeasurement.setMeasurementNoise(this->measNoiseScaling * dcm_CB.transpose() *
                                                       this->starTrackerMessages[index].measurementNoise_C * dcm_CB);
            starTrackerMeasurement.setObservation(
                mrpSwitch(Eigen::Map<Eigen::Vector3d>(starTracker.MRP_BdyInrtl).eval(), this->mrpSwitchThreshold));
            starTrackerMeasurement.setMeasurementModel(MeasurementModel::positionStates);

            std::function<const Eigen::VectorXd(const Eigen::Vector3d&, const Eigen::Vector3d&)> mrpSub =
                [](Eigen::Vector3d const& observed, const Eigen::Vector3d& predicted) {
                    Eigen::Vector3d yMeas = observed - predicted;
                    if (observed.norm() > 0.95 && predicted.norm() > 0.95) {
                        const Eigen::Vector3d predictedShadow = mrpShadow(predicted);
                        Eigen::Vector3d yMeasShadow = observed - predictedShadow;
                        if (yMeasShadow.norm() < yMeas.norm()) {
                            return yMeasShadow;
                        }
                    }
                    return yMeas;
                };
            starTrackerMeasurement.setMeasurementSubtraction(mrpSub);

            this->measurements[this->measurementIndex] = starTrackerMeasurement;
            this->measurementIndex += 1;
            this->validStarTracker = true;
            /*! - Only consider the filter started once a Star Tracker image is processed */
            if (this->firstFilterPass) {
                this->firstFilterPass = false;
            }
        } else {
            this->validStarTracker = false;
        }
    }
}

/*! Loop through the entire gyro buffer to find the first index that is in the future compared to the
 * previousFilterTimeTag. This does not assume the data comes in chronological order since the gyro data
 * is a ring buffer and can wrap around
 * @return void
 * */
void InertialAttitudeUkf::readGyroData() {
    IMUSensorMsgPayload gyroBuffer = this->imuSensorDataInMsg();
    if (gyroBuffer.timeTag > this->previousFilterTimeTag) {
        auto gyroMeasurement = MeasurementModel();
        gyroMeasurement.setMeasurementName("gyro");
        gyroMeasurement.setTimeTag(gyroBuffer.timeTag);
        gyroMeasurement.setValidity(true);

        gyroMeasurement.setMeasurementNoise(this->measNoiseScaling * this->gyroNoise /
                                            gyroBuffer.numberOfValidGyroMeasurements);
        gyroMeasurement.setObservation(cArrayToEigenVector(gyroBuffer.AngVelPlatform));
        gyroMeasurement.setMeasurementModel(MeasurementModel::velocityStatesWithBias);
        this->measurements[this->measurementIndex] = gyroMeasurement;
        this->measurementIndex += 1;
    }
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
    readStarTrackerData();
    /*! Only add the gyro measurements to processing if the filter is in a mode that desires that */
    if (this->measurementAcceptanceMethod == AttitudeFilterMethod::AllMeasurements) {
        readGyroData();
    }
    if (measurementAcceptanceMethod == AttitudeFilterMethod::GyroWhenDazzled && !this->validStarTracker) {
        readGyroData();
    }
}

/*! Set the gyro measurement noise matrix
    @param Eigen::Matrix3d gyroNoise
    @return void
    */
void InertialAttitudeUkf::setGyroNoise(const Eigen::Matrix3d& gyroNoiseInput) { this->gyroNoise = gyroNoiseInput; }

/*! Get the gyro measurement noise matrix
    @return Eigen::Matrix3d gyroNoise
    */
Eigen::Matrix3d InertialAttitudeUkf::getGyroNoise() const { return this->gyroNoise; }

/*! Add a star tracker to the filter solution using the StarTrackerMessage class
    @return StarTrackerMessage starTracker
    */
void InertialAttitudeUkf::addStarTrackerInput(const StarTrackerMessage& starTracker) {
    this->starTrackerMessages[this->numberOfStarTackers] = starTracker;
    this->numberOfStarTackers += 1;
}

/*! Get the star tracker measurement noise matrix for a particular number (indexed at 0)
    @param int starTrackerNumber
    @return Eigen::Matrix3d starTrackerMeasurementNoise
    */
Eigen::Matrix3d InertialAttitudeUkf::getStarTrackerNoise(int starTrackerNumber) const {
    assert(starTrackerNumber < this->numberOfStarTackers);
    return this->starTrackerMessages[starTrackerNumber].measurementNoise_C;
}
