/*
 ISC License

 Copyright (c) 2025, University of Colorado at Boulder

 Permission to use, copy, modify, and/or distribute this software for any
 purpose with or without fee is hereby granted, provided that the above
 copyright notice and this permission notice appear in all copies.

 THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
 WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
 MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
 ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
 WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
 ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
 OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.

 */

#include "cobConverter.h"

/**
 * @brief Compute total COB covariance in image space given unit-vector covariances.
 *
 * The covariance contributions include navigation, attitude, and COB measurement terms.
 * They are rotated into the camera frame and mapped to pixel space via the camera
 * calibration matrix K.
 *
 * @param covarNav_N Navigation covariance (inertial frame)
 * @param covarAtt_B Attitude covariance (body frame)
 * @param covarCob_C COB covariance (camera frame)
 * @param dcm_CN DCM from camera to inertial (C->N)
 * @param dcm_CB DCM from camera to body (C->B)
 * @param cameraCalibrationMatrix Camera calibration matrix K
 * @return Image-space covariance matrix in pixel units
 */
static Eigen::Matrix3d computeTotalCobCovariance(const Eigen::Matrix3d& covarNav_N,
                                                 const Eigen::Matrix3d& covarAtt_B,
                                                 const Eigen::Matrix3d& covarCob_C,
                                                 const Eigen::Matrix3d& dcm_CN,
                                                 const Eigen::Matrix3d& dcm_CB,
                                                 const Eigen::Matrix3d& cameraCalibrationMatrix);

/**
 * @brief Construct a CobConverter.
 * @param method Phase-angle correction method to apply.
 * @param radiusObject Object radius in meters (must be > 0).
 * @note The radius is validated with an assertion.
 */
CobConverter::CobConverter(const PhaseAngleCorrectionMethod method, const double radiusObject) {
    this->phaseAngleCorrectionMethod = method;
    assert(radiusObject > 0);
    this->objectRadius = radiusObject;
}

/** @brief Default destructor. */
CobConverter::~CobConverter() = default;

/**
 * @brief Reset internal state and validate required input connections.
 * @param currentSimNanos Current simulation time in nanoseconds.
 * @throws std::invalid_argument If any required input message link is missing.
 */
void CobConverter::reset(uint64_t currentSimNanos) {
    // check that the required message has not been connected
    if (!this->opnavCOBInMsg.isLinked()) {
        throw std::invalid_argument("CobConverter.opnavCOBInMsg wasn't connected.");
    }
    if (this->performOutlierDetection && !this->opnavFilterInMsg.isLinked()) {
        throw std::invalid_argument("CobConverter.opnavFilterInMsg wasn't connected.");
    }
    if (!this->cameraConfigInMsg.isLinked()) {
        throw std::invalid_argument("CobConverter.cameraConfigInMsg wasn't connected.");
    }
    if (!this->navAttInMsg.isLinked()) {
        throw std::invalid_argument("CobConverter.navAttInMsg wasn't connected.");
    }
    if (!this->ephemInMsg.isLinked()) {
        throw std::invalid_argument("CobConverter.ephemInMsg wasn't connected.");
    }
    if (!this->sunInMsg.isLinked()) {
        throw std::invalid_argument("CobConverter.sunInMsg wasn't connected.");
    }

    CameraModelMsgPayload cameraSpecs = this->cameraConfigInMsg();
    this->computeCameraParameters(cameraSpecs);
}

/**
 * @brief Compute camera calibration matrix and camera in body DCM
 *
 * Uses the camera model and navigation attitude to compute:
 *  - Body->Camera (B->C)
 *  - Camera calibration matrix K and its inverse
 *  - Pixel scale, IFOV, and other camera parameters
 *
 * @param cameraSpecs Camera model specifications.
 */
void CobConverter::computeCameraParameters(const CameraModelMsgPayload &cameraSpecs) {
    double sigma_camera[3];
    std::ranges::copy(cameraSpecs.bodyToCameraMrp, std::begin(sigma_camera));
    this->dcm_CB = mrpToDcm(cArray2EigenVector3d(sigma_camera));

    // Camera parameters
    double alpha = 0;
    double fieldOfView = cameraSpecs.fieldOfView[0];
    double resolutionX = cameraSpecs.resolution[0];
    double resolutionY = cameraSpecs.resolution[1];
    double pX = 2. * tan(fieldOfView / 2.0);
    double pY = 2. * tan(fieldOfView * resolutionY / resolutionX / 2.0);
    this->dX = resolutionX / pX;
    double dY = resolutionY / pY;
    double up = resolutionX / 2;
    double vp = resolutionY / 2;
    this->X = 1 / this->dX;
    this->Y = 1 / dY;
    this->ifov_x = fieldOfView / this->dX * pX;
    this->ifov_y = fieldOfView / dY * pY;
    this->cameraId = cameraSpecs.cameraId;

    // Build K and K^{-1}
    this->cameraCalibrationMatrix << this->dX, alpha, up, 0., dY, vp, 0., 0., 1.;
    this->cameraCalibrationMatrixInverse << 1. / this->dX, -alpha / (this->dX * dY), (alpha * vp - dY * up) / (this->dX * dY), 0., 1. / dY,
        -vp / dY, 0., 0., 1.;
}

/**
 * @brief Compute time varying DCMs
 *
 * Uses the camera model and navigation attitude to compute:
 *  - Body->Inertial (B->N) DCMs
 *  - Inertial->Camera (N->C) DCM
 *
 * @param navAttBuffer Navigation attitude buffer containing MRP sigma_BN.
 */
void CobConverter::computeRotations(const NavAttMsgPayload &navAttBuffer) {
    double sigma_BN[3];
    std::ranges::copy(navAttBuffer.sigma_BN, std::begin(sigma_BN));
    // Extract rotations from relevant messages
    this->dcm_BN = mrpToDcm(cArray2EigenVector3d(sigma_BN));
    this->dcm_NC = this->dcm_BN.transpose() * this->dcm_CB.transpose();

}

/**
 * @brief Compute phase-angle correction term and related angles.
 *
 * Depending on the configured method, computes a brightness offset factor @c gamma
 * (Lambertian or Binary) and the sun direction angle @c phi in the image plane.
 * Also sets @c validCOM when a correction is applied.
 *
 * @param sunBuffer Sun-pointing attitude buffer with vehSunPntBdy.
 * @note Computes and stores: @c alphaPA, @c phi, @c gamma, @c spacecraftRange, @c Rc.
 */
void CobConverter::computePhaseAngleCorrection(const EphemerisMsgPayload &ephemBuffer, const NavAttMsgPayload &sunBuffer) {
    double ephem_position[3];
    double sun_B[3];
    std::ranges::copy(ephemBuffer.r_BdyZero_N, std::begin(ephem_position));
    std::ranges::copy(sunBuffer.vehSunPntBdy, std::begin(sun_B));

    this->sc_position = cArray2EigenVector3d(ephem_position);
    Eigen::Vector3d rhat_N = this->sc_position.normalized();
    Eigen::Vector3d shat_B = cArray2EigenVector3d(sun_B).normalized();
    this->shat_N = dcm_BN.transpose() * shat_B;
    Eigen::Vector3d shat_C = dcm_CB * shat_B;

    this->alphaPA = acos(rhat_N.transpose() * this->shat_N);  // phase angle
    this->phi = atan2(shat_C[1], shat_C[0]);  // sun direction in image plane
    if (this->phaseAngleCorrectionMethod == PhaseAngleCorrectionMethod::Lambertian) {
        // Using phase angle correction assuming Lambertian reflectance sphere (Bhaskaran 1998)
        this->gamma = 3.0 * M_PI / 16.0 * ((cos(this->alphaPA) + 1.0) * sin(this->alphaPA)) /
                (sin(this->alphaPA) + (M_PI - this->alphaPA) * cos(this->alphaPA));
        validCOM = true;
    } else if (this->phaseAngleCorrectionMethod == PhaseAngleCorrectionMethod::Binary) {
        // Using phase angle correction assuming a binarized image (brightness either 0 or 1)
        this->gamma = 4.0 / (3.0 * M_PI) * (1.0 - cos(this->alphaPA));
        validCOM = true;
    }
    this->spacecraftRange = this->sc_position.norm();
    this->Rc = this->objectRadius * this->dX / this->spacecraftRange;  // object radius in pixels
}

/**
 * @brief Compute centers of brightness and mass in pixel coordinates.
 * @param cobMsgBuffer COB message payload (pixel-based center of brightness).
 * @return Tuple of (centerOfBrightness, centerOfMass) as 3-vectors in homogeneous pixel coords.
 */
std::tuple<Eigen::Vector3d, Eigen::Vector3d> CobConverter::computeCentersOfInterest(const OpNavCOBMsgPayload &cobMsgBuffer) const {
    // Center of Brightness in pixel space
    Eigen::Vector3d centerOfBrightness;
    centerOfBrightness[0] = cobMsgBuffer.centerOfBrightness[0];
    centerOfBrightness[1] = cobMsgBuffer.centerOfBrightness[1];
    centerOfBrightness[2] = 1.0;

    // Center of Mass in pixel space (offset by phase-angle correction)
    Eigen::Vector3d centerOfMass;
    centerOfMass[0] = centerOfBrightness[0] - gamma * Rc * cos(phi);
    centerOfMass[1] = centerOfBrightness[1] - gamma * Rc * sin(phi);
    centerOfMass[2] = 1.0;
    return {centerOfBrightness, centerOfMass};
}

/**
 * @brief Compute unit vectors in the camera frame from pixel coordinates.
 * @param centerOfBrightness 3-vector (homogeneous) pixel coordinates of COB.
 * @param centerOfMass 3-vector (homogeneous) pixel coordinates of COM.
 * @note Populates @c rhatCOB_C, @c rhatCOM_C and caches @c rhatCOBNorm.
 */
void CobConverter::computeRelevantVectors(const Eigen::Vector3d &centerOfBrightness, const Eigen::Vector3d &centerOfMass) {
    // Retrieve the vector from target to camera and normalize
    this->rhatCOB_C = -this->cameraCalibrationMatrixInverse * centerOfBrightness;
    this->rhatCOM_C = -this->cameraCalibrationMatrixInverse * centerOfMass;
    this->rhatCOBNorm = rhatCOB_C.norm();
    this->rhatCOB_C.normalize();
    this->rhatCOM_C.normalize();
}

/**
 * @brief Compute the measurement uncertainty in the camera frame.
 *
 * If Binary phase-angle correction is used and a nonzero object radius uncertainty
 * is provided, incorporates the propagated uncertainty of the phase-angle correction.
 * Otherwise, uses a diagonal COB covariance scaled by the number of pixels found,
 * then rotates it into the body frame and adds attitude covariance.
 *
 * @param filterMsgBuffer Filter state and covariance.
 * @param pixelsFound Number of detected pixels (for scale factor).
 */
void CobConverter::computeCameraFrameUncertainty(const FilterMsgPayload &filterMsgBuffer, double pixelsFound) {
    double covariance[MAX_STATES_VECTOR*MAX_STATES_VECTOR];
    std::ranges::copy(filterMsgBuffer.covar, std::begin(covariance));

    // Compute partials of the phase angle and Geometric model correction
    const double scaleFactor = sqrt(pixelsFound / (4 * M_PI)) ;
    this->covar_B.setZero();
    if (phaseAngleCorrectionMethod == PhaseAngleCorrectionMethod::Binary && this->objectRadiusUncertainty > 0) {
        const double constants_deltaR =
            (4 * this->objectRadius / (3 * M_PI * this->sc_position.norm()) * (1 - cos(this->alphaPA)) /
             (1 + pow(4.0 * this->objectRadius / (3.0 * M_PI * this->sc_position.norm()) * (1.0 - cos(this->alphaPA)), 2.0)));

        const Eigen::RowVector3d deltaBinary_delta_r = (-this->sc_position.normalized() / this->sc_position.norm() * constants_deltaR);

        const double deltaBinary_delta_R = (constants_deltaR / this->objectRadius);

        const double deltaBinary_deltaAlpha =
            (4 * this->objectRadius / (3 * M_PI * this->sc_position.norm()) /
             (1 + pow(4.0 * this->objectRadius / (3.0 * M_PI * this->sc_position.norm()) * (1.0 - cos(this->alphaPA)), 2.0)));

        const Eigen::Matrix<double, 3, 3> I = Eigen::Matrix3d::Identity();
        const Eigen::RowVector3d sr = this->shat_N / this->sc_position.norm();
        const Eigen::Matrix<double, 3, 3> rr = I - (this->sc_position.normalized() * this->sc_position.normalized().transpose());
        const Eigen::RowVector3d deltaAlpha_delta_R = sr * rr;

        // Compute COM uncertainty direction
        const Eigen::Matrix<double, 6, 6> Covariance = cArray2EigenMatrixXd(covariance, filterMsgBuffer.numberOfStates, filterMsgBuffer.numberOfStates);
        const Eigen::Matrix3d positionCovariance = Covariance.topLeftCorner(3, 3);

        const Eigen::RowVector3d deltaBinary_r = deltaBinary_delta_r + (deltaBinary_deltaAlpha * deltaAlpha_delta_R);

        double total_deltaBinary_partials = deltaBinary_r * positionCovariance * deltaBinary_r.transpose();
        double term2 = pow(deltaBinary_delta_R, 2) * pow(this->objectRadiusUncertainty, 2);
        double sigma_beta_squared = total_deltaBinary_partials + term2;

        // Define diagonal COM covariance in C and rotate to B
        Eigen::Matrix3d covarCom_C = Eigen::Matrix3d::Zero();
        covarCom_C.setZero();
        covarCom_C(0, 0) = pow(this->X, 2) + sigma_beta_squared / pow(this->ifov_x, 2) * cos(this->phi);
        covarCom_C(1, 1) = pow(this->Y, 2) + sigma_beta_squared / pow(this->ifov_y, 2) * sin(this->phi);
        covarCom_C(2, 2) = 1;
        covarCom_C *= scaleFactor;
        const Eigen::Matrix3d covarCom_B = this->dcm_CB.transpose() * covarCom_C * this->dcm_CB;

        // Add COM covariance in B frame to get total covariance
        this->covar_B = covarCom_B + this->covarAtt_BN_B;

    } else {
        // Define diagonal COB covariance
        Eigen::Matrix3d covarCob_C;
        covarCob_C.setZero();
        covarCob_C(0, 0) = pow(this->X, 2);
        covarCob_C(1, 1) = pow(this->Y, 2);
        covarCob_C(2, 2) = 1;
        // Scale by number of pixels and rotate to B
        covarCob_C *= scaleFactor;
        const Eigen::Matrix3d covarCom_B = this->dcm_CB.transpose() * covarCob_C * this->dcm_CB;
        this->covar_B = covarCom_B + this->covarAtt_BN_B;
    }
}

/**
 * @brief Populate and return output message payloads for COB, COM, and COM metadata.
 *
 * @param timeTag Measurement timestamp (nanoseconds).
 * @param centerOfMass COM in pixel coordinates (homogeneous).
 * @param uVecCOBMsgBuffer Output COB unit-vector payload (to be filled).
 * @param uVecCOMMsgBuffer Output COM unit-vector payload (to be filled).
 * @param comMsgBuffer Output COM metadata payload (to be filled).
 * @return Tuple of populated (uVecCOBMsgBuffer, uVecCOMMsgBuffer, comMsgBuffer).
 */
std::tuple<OpNavUnitVecMsgPayload, OpNavUnitVecMsgPayload,OpNavCOMMsgPayload> CobConverter::populateOutputMessages(const uint64_t timeTag, const Eigen::Vector3d &centerOfMass, OpNavUnitVecMsgPayload &uVecCOBMsgBuffer, OpNavUnitVecMsgPayload &uVecCOMMsgBuffer, OpNavCOMMsgPayload &comMsgBuffer) {

    Eigen::Vector3d rhatCOB_N = this->dcm_NC*this->rhatCOB_C;
    Eigen::Vector3d rhatCOB_B = this->dcm_BN*rhatCOB_N;
    Eigen::Vector3d rhatCOM_N = this->dcm_NC*this->rhatCOM_C;
    Eigen::Vector3d rhatCOM_B = this->dcm_BN*rhatCOB_N;
    Eigen::Matrix3d covar_N = this->dcm_BN.transpose() * this->covar_B * this->dcm_BN;
    Eigen::Matrix3d covar_C = this->dcm_NC.transpose() * covar_N * this->dcm_NC;

    eigenMatrix3d2CArray(covar_N, uVecCOBMsgBuffer.covar_N);
    eigenMatrix3d2CArray(covar_C, uVecCOBMsgBuffer.covar_C);
    eigenMatrix3d2CArray(this->covar_B, uVecCOBMsgBuffer.covar_B);
    eigenVector3d2CArray(rhatCOB_N, uVecCOBMsgBuffer.rhat_BN_N);
    eigenVector3d2CArray(this->rhatCOB_C, uVecCOBMsgBuffer.rhat_BN_C);
    eigenVector3d2CArray(rhatCOB_B, uVecCOBMsgBuffer.rhat_BN_B);
    uVecCOBMsgBuffer.timeTag = static_cast<double>(timeTag) * NANO2SEC;
    uVecCOBMsgBuffer.valid = this->goodOutlierCheck;

    eigenMatrix3d2CArray(covar_N, uVecCOMMsgBuffer.covar_N);
    eigenMatrix3d2CArray(covar_C, uVecCOMMsgBuffer.covar_C);
    eigenMatrix3d2CArray(this->covar_B, uVecCOMMsgBuffer.covar_B);
    eigenVector3d2CArray(rhatCOM_N, uVecCOMMsgBuffer.rhat_BN_N);
    eigenVector3d2CArray(this->rhatCOM_C, uVecCOMMsgBuffer.rhat_BN_C);
    eigenVector3d2CArray(rhatCOM_B, uVecCOMMsgBuffer.rhat_BN_B);
    uVecCOMMsgBuffer.timeTag = static_cast<double>(timeTag) * NANO2SEC;
    uVecCOMMsgBuffer.valid = (this->validCOM && this->goodOutlierCheck);

    comMsgBuffer.centerOfMass[0] = centerOfMass[0];
    comMsgBuffer.centerOfMass[1] = centerOfMass[1];
    comMsgBuffer.offsetFactor = this->gamma;
    comMsgBuffer.objectPixelRadius = static_cast<int>(this->Rc);
    comMsgBuffer.phaseAngle = this->alphaPA;
    comMsgBuffer.sunDirection = this->phi;
    comMsgBuffer.cameraID = this->cameraId;
    comMsgBuffer.timeTag = timeTag;
    comMsgBuffer.valid = this->validCOM;

    return {uVecCOBMsgBuffer, uVecCOMMsgBuffer, comMsgBuffer};
}

/**
 * @brief Update step: convert pixel-based COB into unit vectors and outputs.
 *
 * Reads inputs, computes parameters and corrections, performs optional outlier
 * detection, and writes out three payloads: COB unit vector, COM unit vector, and
 * COM metadata.
 *
 * @param currentSimNanos Current simulation time in nanoseconds.
 */
void CobConverter::updateState(const uint64_t currentSimNanos) {
    OpNavCOBMsgPayload cobMsgBuffer = this->opnavCOBInMsg();
    NavAttMsgPayload navAttBuffer = this->navAttInMsg();
    EphemerisMsgPayload ephemBuffer = this->ephemInMsg();
    NavAttMsgPayload sunBuffer = this->sunInMsg();
    FilterMsgPayload filterMsgBuffer = this->opnavFilterInMsg();

    OpNavUnitVecMsgPayload uVecCOBMsgBuffer{};
    OpNavUnitVecMsgPayload uVecCOMMsgBuffer{};
    OpNavCOMMsgPayload comMsgBuffer{};

    if (cobMsgBuffer.valid && cobMsgBuffer.pixelsFound != 0) {
        this->computeRotations(navAttBuffer);

        // Phase angle correction
        if (this->phaseAngleCorrectionMethod != PhaseAngleCorrectionMethod::NoCorrection) {
            this->validCOM = true;
            this->computePhaseAngleCorrection(ephemBuffer, sunBuffer);
        }

        auto [centerOfBrightness, centerOfMass] = this->computeCentersOfInterest(cobMsgBuffer);
        this->computeRelevantVectors(centerOfBrightness, centerOfMass);
        this->computeCameraFrameUncertainty(filterMsgBuffer, cobMsgBuffer.pixelsFound);

        if (this->performOutlierDetection) {
            this->cobOutlierDetection(filterMsgBuffer);
        }

        std::tie(uVecCOBMsgBuffer, uVecCOMMsgBuffer, comMsgBuffer) = this->populateOutputMessages(cobMsgBuffer.timeTag, centerOfMass, uVecCOBMsgBuffer, uVecCOMMsgBuffer, comMsgBuffer);
    }

    this->opnavUnitVecCOBOutMsg.write(&uVecCOBMsgBuffer, this->moduleID, currentSimNanos);
    this->opnavUnitVecCOMOutMsg.write(&uVecCOMMsgBuffer, this->moduleID, currentSimNanos);
    this->opnavCOMOutMsg.write(&comMsgBuffer, this->moduleID, currentSimNanos);
}

/**
 * @brief Helper to combine nav, attitude, and COB covariances and map to image space.
 *
 * @param covarNav_N Navigation covariance (inertial frame).
 * @param covarAtt_B Attitude covariance (body frame).
 * @param covarCob_C COB covariance (camera frame).
 * @param dcm_CN DCM camera-to-inertial.
 * @param dcm_CB DCM camera-to-body.
 * @param cameraCalibrationMatrix Camera calibration matrix K.
 * @return Image-space covariance (pixels).
 */
static Eigen::Matrix3d computeTotalCobCovariance(const Eigen::Matrix3d& covarNav_N,
                                                 const Eigen::Matrix3d& covarAtt_B,
                                                 const Eigen::Matrix3d& covarCob_C,
                                                 const Eigen::Matrix3d& dcm_CN,
                                                 const Eigen::Matrix3d& dcm_CB,
                                                 const Eigen::Matrix3d& cameraCalibrationMatrix) {
    Eigen::Matrix3d covarAtt_C = dcm_CB * covarAtt_B * dcm_CB.transpose();
    Eigen::Matrix3d covarNav_C = dcm_CN * covarNav_N * dcm_CN.transpose();
    Eigen::Matrix3d covarTotal_C = covarCob_C + covarAtt_C + covarNav_C;
    Eigen::Matrix3d covarImage = cameraCalibrationMatrix * covarTotal_C * cameraCalibrationMatrix.transpose();

    return covarImage;
}

/**
 * @brief Perform outlier detection on the COB measurement.
 *
 * Projects the filter's expected unit vector to pixel space and compares against the
 * measured COB. Uses either a specified standard deviation or one derived from the
 * combined image covariance to perform a sigma-based gate.
 *
 * @param filterMsgBuffer Filter message buffer containing state and covariance.
 */
void CobConverter::cobOutlierDetection(const FilterMsgPayload &filterMsgBuffer) {
    double state[MAX_STATES_VECTOR];
    double covariance[MAX_STATES_VECTOR*MAX_STATES_VECTOR];
    std::ranges::copy(filterMsgBuffer.state, std::begin(state));
    std::ranges::copy(filterMsgBuffer.covar, std::begin(covariance));

    int numberOfStates = filterMsgBuffer.numberOfStates;
    Eigen::VectorXd filterState = cArray2EigenMatrixXd(state, numberOfStates, 1);
    Eigen::Vector3d rNav_BN_N = filterState.segment(0, 3);
    Eigen::Vector3d rhatNav_N = rNav_BN_N.normalized();
    Eigen::MatrixXd filterCovariance =
        cArray2EigenMatrixXd(covariance, numberOfStates, numberOfStates);
    Eigen::Matrix3d covarNav_N = filterCovariance.block(0, 0, 3, 3) / pow(rNav_BN_N.norm(), 2);

    Eigen::Vector3d rhatCOB_C = - this->rhatCOB_C;            // turn unit vector from asteroid to camera into unit vector from camera to asteroid
    rhatCOB_C /= rhatCOB_C[2];  // make z-component 1 for image plane
    Eigen::Vector3d cob = this->cameraCalibrationMatrix * rhatCOB_C;

    // assume that the time of the last filter update corresponds to the current timestep (so no propagation required)
    Eigen::Vector3d rhatNav_C = (this->dcm_NC.transpose() * rhatNav_N);
    rhatNav_C *= -1;
    rhatNav_C /= rhatNav_C[2];
    Eigen::Vector3d cobNav = this->cameraCalibrationMatrix * rhatNav_C;

    double cobErrorPrediction = (cob - cobNav).norm();
    double sigma;
    if (this->specifiedStandardDeviation) {
        sigma = this->standardDeviation;
    } else {
        Eigen::Matrix3d covarImage = computeTotalCobCovariance(
            covarNav_N, this->covarAtt_BN_B, this->dcm_CB.transpose()*this->covar_B*this->dcm_CB.transpose(), this->dcm_NC.transpose(), this->dcm_CB, this->cameraCalibrationMatrix);
        sigma = sqrt(std::max(covarImage(0, 0), covarImage(1, 1)));
    }

    this->goodOutlierCheck = cobErrorPrediction < this->numStandardDeviations * sigma;
}

/**
 * @brief Set the object radius.
 * @param radius Object radius in meters (must be > 0).
 */
void CobConverter::setRadius(const double radius) {
    assert(radius > 0);
    this->objectRadius = radius;
}

/**
 * @brief Get the object radius.
 * @return Object radius in meters.
 */
double CobConverter::getRadius() const { return this->objectRadius; }

/**
 * @brief Set the object radius uncertainty.
 * @param radiusUncertainty Object radius uncertainty in meters (>= 0).
 */
void CobConverter::setRadiusUncertainty(const double radiusUncertainty) {
    assert(radiusUncertainty >= 0);
    this->objectRadiusUncertainty = radiusUncertainty;
}

/**
 * @brief Get the object radius uncertainty.
 * @return Object radius uncertainty in meters.
 */
double CobConverter::getRadiusUncertainty() const { return this->objectRadiusUncertainty; }

/**
 * @brief Set the attitude error covariance matrix in body frame (for unit vector measurements).
 * @param covAtt_BN_B 3x3 attitude covariance in body frame.
 */
void CobConverter::setAttitudeCovariance(const Eigen::Matrix3d &covAtt_BN_B) { this->covarAtt_BN_B = covAtt_BN_B; }

/**
 * @brief Get the attitude error covariance matrix in body frame (for unit vector measurements).
 * @return 3x3 attitude covariance in body frame.
 */
Eigen::Matrix3d CobConverter::getAttitudeCovariance() const { return this->covarAtt_BN_B; }

/**
 * @brief Set the number of standard deviations for outlier gating.
 * @param num Number of sigmas (> 0).
 */
void CobConverter::setNumStandardDeviations(const double num) {
    assert(num > 0.0);
    this->numStandardDeviations = num;
}

/**
 * @brief Get the configured number of standard deviations for outlier gating.
 * @return Number of sigmas.
 */
double CobConverter::getNumStandardDeviations() const { return this->numStandardDeviations; }

/**
 * @brief Set an explicit standard deviation for the expected COB error.
 * @param num Standard deviation (> 0).
 * @note When set, outlier detection will use this fixed value instead of deriving one.
 */
void CobConverter::setStandardDeviation(const double num) {
    assert(num > 0.0);
    this->standardDeviation = num;
    this->specifiedStandardDeviation = true;
}

/**
 * @brief Get the explicitly specified standard deviation (if set).
 * @return Standard deviation value.
 */
double CobConverter::getStandardDeviation() const { return this->standardDeviation; }

/**
 * @brief Determine whether a standard deviation has been explicitly specified.
 * @return True if specified, false otherwise.
 */
bool CobConverter::isStandardDeviationSpecified() const { return this->specifiedStandardDeviation; }

/**
 * @brief Enable COB outlier detection.
 */
void CobConverter::enableOutlierDetection() { this->performOutlierDetection = true; }

/**
 * @brief Disable COB outlier detection.
 */
void CobConverter::disableOutlierDetection() { this->performOutlierDetection = false; }

/**
 * @brief Check whether COB outlier detection is enabled.
 * @return True if enabled, false otherwise.
 */
bool CobConverter::isOutlierDetectionEnabled() const { return this->performOutlierDetection; }
