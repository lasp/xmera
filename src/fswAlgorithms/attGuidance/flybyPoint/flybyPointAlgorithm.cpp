// SPDX-License-Identifier: ISC
// Copyright (c) 2025, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

#include "flybyPointAlgorithm.h"
#include <architecture/utilities/eigenSupport.h>
#include <architecture/utilities/macroDefinitions.h>
#include <architecture/utilities/rigidBodyKinematics.hpp>

/*! This method is used to reset the module.
 @return void
 */
void FlybyPointAlgorithm::reset() {
    this->lastFilterReadTime = 0;
    this->firstRead = true;
}

/*! This function computes a reference attitude frame for a spacecraft in relative motion about a small body.
 @return AttRefMsgPayload
 @param currentSimNanos The current simulation time for system
 @param r_BN_N The relative position state
 @param v_BN_N The relative velocity state
 */
std::pair<AttRefMsgPayload, FlybyDiagnosticMsgPayload> FlybyPointAlgorithm::updateState(uint64_t currentSimNanos,
                                                                                        const Eigen::Vector3d& r_BN_N,
                                                                                        const Eigen::Vector3d& v_BN_N) {
    /*! init diagnostic message */
    FlybyDiagnosticMsgPayload flybyDiagnosticMsgBuffer = {false, false, false, false};
    /*! compute dt from current time and last filter read time and get new states*/
    this->dt = (currentSimNanos - this->lastFilterReadTime) * NANO2SEC;
    if ((this->dt >= this->timeBetweenFilterData) || this->firstRead) {
        /*! If this is the first read, seed the algorithm with the solution  */
        if (this->firstRead) {
            this->timeOfFirstRead = currentSimNanos * NANO2SEC;
            this->firstNavPosition = r_BN_N;
            this->firstNavVelocity = v_BN_N;
            this->computeFlybyParameters(r_BN_N, v_BN_N);
            this->computeRN(r_BN_N, v_BN_N);
            this->firstRead = false;
        }
        /*! Protect against bad new solutions by checking validity */
        else if (this->checkValidity(currentSimNanos, r_BN_N, v_BN_N, flybyDiagnosticMsgBuffer)) {
            /*! update flyby parameters and guidance frame */
            this->computeFlybyParameters(r_BN_N, v_BN_N);
            this->computeRN(r_BN_N, v_BN_N);

            /*! update lastFilterReadTime to current time and dt to zero */
            this->lastFilterReadTime = currentSimNanos;
            this->dt = 0;
        }
    }
    auto [sigma_RN, omega_RN_N, omegaDot_RN_N] = this->computeGuidanceSolution();
    AttRefMsgPayload attMsgBuffer{};
    eigenVectorToCArray(sigma_RN, attMsgBuffer.sigma_RN);
    eigenVectorToCArray(omega_RN_N, attMsgBuffer.omega_RN_N);
    eigenVectorToCArray(omegaDot_RN_N, attMsgBuffer.domega_RN_N);
    return {attMsgBuffer, flybyDiagnosticMsgBuffer};
}

void FlybyPointAlgorithm::computeFlybyParameters(const Eigen::Vector3d& r_BN_N, const Eigen::Vector3d& v_BN_N) {
    this->f0 = v_BN_N.norm() / r_BN_N.norm();

    /*! compute radial (ur_N), velocity (uv_N), along-track (ut_N), and out-of-plane (uh_N) unit direction vectors */
    Eigen::Vector3d ur_N = r_BN_N.normalized();
    Eigen::Vector3d uv_N = v_BN_N.normalized();

    Eigen::Vector3d uh_N = ur_N.cross(uv_N).normalized();
    Eigen::Vector3d ut_N = uh_N.cross(ur_N).normalized();

    // compute flight path angle at the time of read
    this->gamma0 = std::atan(v_BN_N.dot(ur_N) / v_BN_N.dot(ut_N));
}

bool FlybyPointAlgorithm::checkValidity(uint64_t currentSimNanos,
                                        const Eigen::Vector3d& r_BN_N,
                                        const Eigen::Vector3d& v_BN_N,
                                        FlybyDiagnosticMsgPayload& flybyDiagnosticMsgBuffer) const {
    bool valid = true;
    Eigen::Vector3d ur_N = r_BN_N.normalized();
    Eigen::Vector3d uv_N = v_BN_N.normalized();

    /*! assert r and v are not collinear (collision trajectory) */
    if (std::abs(1 - ur_N.dot(uv_N)) < this->toleranceForCollinearity) {
        valid = false;
        flybyDiagnosticMsgBuffer.collinearityTrigger = true;
    } else {
        flybyDiagnosticMsgBuffer.collinearityTrigger = false;
    }

    /*! check if the predicted rate exceeds the maximum rate of the spacecraft */
    double distanceClosestApproach = -r_BN_N.norm() * std::sin(this->gamma0);
    double maxPredictedRate = v_BN_N.norm() / distanceClosestApproach * 180 / M_PI;
    if (maxPredictedRate > this->maxRate && this->maxRate > 0) {
        valid = false;
        flybyDiagnosticMsgBuffer.maxRateTrigger = true;
    } else {
        flybyDiagnosticMsgBuffer.maxRateTrigger = false;
    }

    /*! check if the predicted acceleration exceeds the maximum acceleration of the spacecraft */
    double maxPredictedAcceleration =
        3 * std::sqrt(3) / 8 * pow(v_BN_N.norm() / distanceClosestApproach, 2) * 180 / M_PI;
    if (maxPredictedAcceleration > this->maxAcceleration && this->maxAcceleration > 0) {
        valid = false;
        flybyDiagnosticMsgBuffer.maxAccelerationTrigger = true;
    } else {
        flybyDiagnosticMsgBuffer.maxAccelerationTrigger = false;
    }

    /*! check if the position error exceeds a-priori sigma bound */
    double deltaT = currentSimNanos * NANO2SEC - this->timeOfFirstRead;
    double deltaPositionNorm = (r_BN_N - (this->firstNavPosition + deltaT * this->firstNavVelocity)).norm();
    if (deltaPositionNorm > this->positionKnowledgeSigma && this->positionKnowledgeSigma > 0) {
        valid = false;
        flybyDiagnosticMsgBuffer.positionKnowledgeExceedTrigger = true;
    } else {
        flybyDiagnosticMsgBuffer.positionKnowledgeExceedTrigger = false;
    }

    return valid;
}

void FlybyPointAlgorithm::computeRN(const Eigen::Vector3d& r_BN_N, const Eigen::Vector3d& v_BN_N) {
    /*! compute radial (ur_N), velocity (uv_N), along-track (ut_N), and out-of-plane (uh_N) unit direction vectors */
    Eigen::Vector3d ur_N = r_BN_N.normalized();
    Eigen::Vector3d uv_N = v_BN_N.normalized();

    Eigen::Vector3d uh_N = ur_N.cross(uv_N).normalized();
    Eigen::Vector3d ut_N = uh_N.cross(ur_N).normalized();

    /*! compute inertial-to-reference DCM at time of read */
    this->R0N.row(0) = ur_N;
    this->R0N.row(1) = ut_N;
    this->R0N.row(2) = uh_N;
}

std::tuple<Eigen::Vector3d, Eigen::Vector3d, Eigen::Vector3d> FlybyPointAlgorithm::computeGuidanceSolution() const {
    /*! compute DCM (RtR0) of reference frame from last read time */
    double theta = std::atan(std::tan(this->gamma0) + this->f0 / std::cos(this->gamma0) * this->dt) - this->gamma0;
    Eigen::Vector3d PRV_theta{0, 0, theta};
    Eigen::Matrix3d RtR0 = prvToDcm(PRV_theta);

    /*! compute DCM of reference frame at time t_0 + dt with respect to inertial frame */
    Eigen::Matrix3d RtN = RtR0 * this->R0N;

    /*! compute scalar angular rate and acceleration of the reference frame in R-frame coordinates */
    double den = (this->f0 * this->f0 * this->dt * this->dt + 2 * this->f0 * sin(this->gamma0) * this->dt + 1);
    double thetaDot = this->f0 * cos(this->gamma0) / den;
    double thetaDDot =
        -2 * this->f0 * this->f0 * cos(this->gamma0) * (this->f0 * this->dt + sin(this->gamma0)) / (den * den);
    Eigen::Vector3d omega_RN_R{0, 0, thetaDot};
    Eigen::Vector3d omegaDot_RN_R{0, 0, thetaDDot};

    /*! populate attRefOut with reference frame information */
    Eigen::Vector3d sigma_RN = dcmToMrp(RtN);

    if (this->signOfOrbitNormalFrameVector == -1) {
        Eigen::Vector3d halfRotationX{1, 0, 0};
        sigma_RN = addMrp(sigma_RN, halfRotationX);
    }
    Eigen::Vector3d omega_RN_N = RtN.transpose() * omega_RN_R;
    Eigen::Vector3d omegaDot_RN_N = RtN.transpose() * omegaDot_RN_R;

    return {sigma_RN, omega_RN_N, omegaDot_RN_N};
}

double FlybyPointAlgorithm::getTimeBetweenFilterData() const { return this->timeBetweenFilterData; }

void FlybyPointAlgorithm::setTimeBetweenFilterData(double time) { this->timeBetweenFilterData = time; }

double FlybyPointAlgorithm::getToleranceForCollinearity() const { return this->toleranceForCollinearity; }

void FlybyPointAlgorithm::setToleranceForCollinearity(double tolerance) { this->toleranceForCollinearity = tolerance; }

/*! Get the sign (+1 or -1) of the axis of rotation of the Z axis during the flyby
 @param int sign (+1 or -1)
 */
int FlybyPointAlgorithm::getSignOfOrbitNormalFrameVector() const { return this->signOfOrbitNormalFrameVector; }

/*! Set the sign (+1 or -1) of the axis of rotation of the Z axis during the flyby
 @param int sign (+1 or -1)
 */
void FlybyPointAlgorithm::setSignOfOrbitNormalFrameVector(int sign) { this->signOfOrbitNormalFrameVector = sign; }

/*! Get the maximum acceleration threshold to consider a solution invalid
 @return double maximum accceleration
 */
double FlybyPointAlgorithm::getMaximumAccelerationThreshold() const { return this->maxAcceleration; }

/*! Set the maximum acceleration threshold to consider a solution invalid
 @param double maximum accceleration
 */
void FlybyPointAlgorithm::setMaximumAccelerationThreshold(double maxAccelerationThreshold) {
    this->maxAcceleration = maxAccelerationThreshold;
}

/*! Get the maximum rate threshold to consider a solution invalid
 @return maximum rate
 */
double FlybyPointAlgorithm::getMaximumRateThreshold() const { return this->maxRate; }

/*! Set the maximum rate threshold to consider a solution invalid
 @param maximum rate
 */
void FlybyPointAlgorithm::setMaximumRateThreshold(double maxRateThreshold) { this->maxRate = maxRateThreshold; }

/*! Get the ground based positional knowledge standard deviation
 @return sigma
 */
double FlybyPointAlgorithm::getPositionKnowledgeSigma() const { return this->positionKnowledgeSigma; }

/*! Set the ground based positional knowledge sigma
 @param sigma
 */
void FlybyPointAlgorithm::setPositionKnowledgeSigma(double positionKnowledgeStd) {
    this->positionKnowledgeSigma = positionKnowledgeStd;
}
