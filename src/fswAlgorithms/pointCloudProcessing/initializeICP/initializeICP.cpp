// SPDX-License-Identifier: ISC
// Copyright (c) 2023, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

#include "initializeICP.h"

InitializeICP::InitializeICP() = default;

InitializeICP::~InitializeICP() = default;

/*! This method performs a complete reset of the module.  Local module variables that retain time varying states
 * between function calls are reset to their default values.
 @return void
 @param currentSimNanos The clock time at which the function was called (nanoseconds)
 */
void InitializeICP::reset(uint64_t currentSimNanos) {
    if (!this->inputMeasuredPointCloud.isLinked()) {
        bskLogger.bskLog(BSK_ERROR, "Measured Point Cloud wasn't connected.");
    }
    if (!this->ephemerisInMsg.isLinked()) {
        bskLogger.bskLog(BSK_ERROR, "Ephemeris message wasn't connected.");
    }
    if (!this->cameraConfigInMsg.isLinked()) {
        bskLogger.bskLog(BSK_ERROR, "Camera message was not linked.");
    }

    //! If the module is reset, use the ephemeris message for initialization
    this->initialPhase = true;
}

/*! Normalize the point cloud with the average norm of all points.
 @return void
 */
void InitializeICP::normalizePointCloud() {
    PointCloudMsgPayload measuredCloudBuffer = this->inputMeasuredPointCloud();
    this->normalizedCloudBuffer = PointCloudMsgPayload{};
    Eigen::MatrixXd measuredPoints =
        cArrayToEigenMatrixX(measuredCloudBuffer.points, SICP_POINT_DIM, measuredCloudBuffer.numberOfPoints);
    Eigen::MatrixXd normalizedPoints = Eigen::MatrixXd::Zero(SICP_POINT_DIM, measuredCloudBuffer.numberOfPoints);
    //! If there is a valid point cloud present, average the point norms to normalize each point
    if (measuredCloudBuffer.valid && measuredCloudBuffer.numberOfPoints > 0) {
        this->averageNorm = 0;
        for (int i = 0; i < measuredCloudBuffer.numberOfPoints; i++) {
            this->averageNorm += measuredPoints.col(i).norm();
        }
        this->averageNorm = this->averageNorm / measuredCloudBuffer.numberOfPoints;
        if (this->normalizeMeasuredCloud) {
            normalizedPoints = measuredPoints / this->averageNorm;
            eigenMatrixXToCArray(normalizedPoints.transpose(), this->normalizedCloudBuffer.points);
        } else {
            eigenMatrixXToCArray(measuredPoints.transpose(), this->normalizedCloudBuffer.points);
        }
    } else {
        this->normalizedCloudBuffer = PointCloudMsgPayload{};
    }

    this->normalizedCloudBuffer.valid = measuredCloudBuffer.valid;
    this->normalizedCloudBuffer.numberOfPoints = measuredCloudBuffer.numberOfPoints;
}

/*! Set initial conditions either with the ephemeris information of the spacecraft, or the previous ICP iteration
 * depending on the initialPhase status
 @return void
 */
void InitializeICP::setInitialConditions(uint64_t currentSimNanos) {
    CameraConfigMsgPayload cameraBuffer = this->cameraConfigInMsg();
    SICPMsgPayload sicpBuffer = this->inputSICPData();

    //!< Allocate appropriate memory to the arrays that will be populated
    Eigen::MatrixXd R_prev = Eigen::MatrixXd::Identity(SICP_POINT_DIM, SICP_POINT_DIM);
    Eigen::MatrixXd t_prev = Eigen::VectorXd::Zero(SICP_POINT_DIM);
    double s_prev = 1;

    //!< When a valid ICP solution has been computed, use that instead of ephemeris information as a priority
    if (sicpBuffer.valid) {
        this->R_logged = cArrayToEigenMatrixX(
            &sicpBuffer.rotationMatrix[(sicpBuffer.numberOfIteration - 1) * SICP_POINT_DIM * SICP_POINT_DIM],
            SICP_POINT_DIM,
            SICP_POINT_DIM);
        this->t_logged = cArrayToEigenMatrixX(
            &sicpBuffer.translation[(sicpBuffer.numberOfIteration - 1) * SICP_POINT_DIM], 1, SICP_POINT_DIM);
        this->s_logged = sicpBuffer.scaleFactor[sicpBuffer.numberOfIteration - 1];
        this->initialPhase = false;
        this->previousTimeTag = sicpBuffer.timeTag;
    }
    double timeSinceICPSolution = (double)(currentSimNanos - this->previousTimeTag) * 1E-9;
    //! - If the current point cloud is valid check if there is a recent ICP solution to use. If there isn't use
    //! ephemeris information
    if (this->normalizedCloudBuffer.valid) {
        if (this->initialPhase || timeSinceICPSolution > this->maxTimeBetweenMeasurements) {
            EphemerisMsgPayload ephemerisInMsgBuffer = this->ephemerisInMsg();
            Eigen::Vector3d r_BN_N = cArrayToEigenVector(ephemerisInMsgBuffer.r_BdyZero_N);

            Eigen::MRPd sigma_CB = cArrayToEigenMrp(cameraBuffer.sigma_CB);
            Eigen::Matrix3d dcm_CB = sigma_CB.toRotationMatrix().transpose();

            Eigen::MRPd sigma_BN = cArrayToEigenMrp(ephemerisInMsgBuffer.sigma_BN);
            Eigen::Matrix3d dcm_BN = sigma_BN.toRotationMatrix().transpose();

            R_prev = (dcm_CB * dcm_BN);
            t_prev = r_BN_N;
            s_prev = 1;
            this->outputIcpBuffer.valid = true;
        } else {
            R_prev = this->R_logged;
            t_prev = this->t_logged;
            s_prev = this->s_logged;
            this->outputIcpBuffer.valid = true;
        }
    } else {
        this->outputIcpBuffer = SICPMsgPayload{};
    }

    this->outputIcpBuffer.scaleFactor[0] = s_prev;
    eigenMatrixXToCArray(R_prev, this->outputIcpBuffer.rotationMatrix);
    eigenMatrixXToCArray(t_prev, this->outputIcpBuffer.translation);
    this->outputIcpBuffer.numberOfIteration = 0;
}

/*! Write out the messages with the transformed data
 @return void
 @param currentSimNanos The clock time at which the function was called (nanoseconds)
 */
void InitializeICP::writeOutputMessages(uint64_t currentSimNanos) {
    //! - Write the algorithm output data with zeros are results
    this->measuredPointCloud.write(&this->normalizedCloudBuffer, this->moduleID, currentSimNanos);
    this->initializeSICPMsg.write(&this->outputIcpBuffer, this->moduleID, currentSimNanos);
}

/*! This module reads a point cloud and performs an normalization on the points, it then reads the last messages
 * containing attitude and position information to seed the ICP algorithm that follows, or uses the
 @return void
 @param currentSimNanos The clock time at which the function was called (nanoseconds)
 */
void InitializeICP::updateState(uint64_t currentSimNanos) {
    //! - Normalize the measured point cloud if it is valid
    this->normalizePointCloud();
    //! - Use previous ICP solution (if previous solution was valid) or spacecraft ephemeris otherwise
    this->setInitialConditions(currentSimNanos);
    //! - Write output messages
    this->writeOutputMessages(currentSimNanos);
}
