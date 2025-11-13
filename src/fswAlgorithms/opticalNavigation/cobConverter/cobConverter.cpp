#include "cobConverter.h"

/**
 * @brief Construct a CobConverter.
 * @param method Phase-angle correction method to apply.
 * @param radiusObject Object radius in meters (must be > 0).
 * @note The radius is validated with an assertion.
 */
CobConverter::CobConverter(const PhaseAngleCorrectionMethod method, const double radiusObject)
    : algorithm(enumMap.at(method), radiusObject) {}

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
    if (this->algorithm.isOutlierDetectionEnabled() && !this->opnavFilterInMsg.isLinked()) {
        throw std::invalid_argument("CobConverter.opnavFilterInMsg wasn't connected.");
    }
    if (!this->cameraConfigInMsg.isLinked()) {
        throw std::invalid_argument("CobConverter.cameraConfigInMsg wasn't connected.");
    }
    if (!this->navAttInMsg.isLinked()) {
        throw std::invalid_argument("CobConverter.navAttInMsg wasn't connected.");
    }
    if (!this->sunInMsg.isLinked()) {
        throw std::invalid_argument("CobConverter.sunInMsg wasn't connected.");
    }
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
    CameraModelMsgPayload cameraModelInMsg = this->cameraConfigInMsg();
    OpNavCOBMsgPayload cobMsgBuffer = this->opnavCOBInMsg();
    NavAttMsgPayload navAttBuffer = this->navAttInMsg();
    NavAttMsgPayload sunBuffer = this->sunInMsg();
    FilterMsgPayload filterMsgBuffer = this->opnavFilterInMsg();

    OpNavUnitVecMsgPayload uVecOutMsgBuffer{};
    OpNavCOMMsgPayload comMsgBuffer{};

    if (cobMsgBuffer.valid && cobMsgBuffer.pixelsFound != 0) {
        std::tie(uVecOutMsgBuffer, comMsgBuffer) = this->algorithm.updateState(
            currentSimNanos, cameraModelInMsg, cobMsgBuffer, navAttBuffer, sunBuffer, filterMsgBuffer);
    }

    this->opnavUnitVecOutMsg.write(&uVecOutMsgBuffer, this->moduleID, currentSimNanos);
    this->comCorrectionOutMsg.write(&comMsgBuffer, this->moduleID, currentSimNanos);
}

/**
 * @brief Set the object radius.
 * @param radius Object radius in meters (must be > 0).
 */
void CobConverter::setRadius(const double radius) {
    assert(radius > 0);
    this->algorithm.setRadius(radius);
}

/**
 * @brief Get the object radius.
 * @return Object radius in meters.
 */
double CobConverter::getRadius() const {
    return this->algorithm.getRadius();
    ;
}

/**
 * @brief Set the object radius uncertainty.
 * @param radiusUncertainty Object radius uncertainty in meters (>= 0).
 */
void CobConverter::setRadiusUncertainty(const double radiusUncertainty) {
    assert(radiusUncertainty >= 0);
    this->algorithm.setRadiusUncertainty(radiusUncertainty);
}

/**
 * @brief Get the object radius uncertainty.
 * @return Object radius uncertainty in meters.
 */
double CobConverter::getRadiusUncertainty() const { return this->algorithm.getRadiusUncertainty(); }

/**
 * @brief Set the attitude error covariance matrix in body frame (for unit vector measurements).
 * @param covAtt_BN_B 3x3 attitude covariance in body frame.
 */
void CobConverter::setAttitudeCovariance(const Eigen::Matrix3d& covAtt_BN_B) {
    this->algorithm.setAttitudeCovariance(covAtt_BN_B);
}

/**
 * @brief Get the attitude error covariance matrix in body frame (for unit vector measurements).
 * @return 3x3 attitude covariance in body frame.
 */
Eigen::Matrix3d CobConverter::getAttitudeCovariance() const { return this->algorithm.getAttitudeCovariance(); }

/**
 * @brief Set the number of standard deviations for outlier gating.
 * @param num Number of sigmas (> 0).
 */
void CobConverter::setNumStandardDeviations(const double num) {
    assert(num > 0.0);
    this->algorithm.setNumStandardDeviations(num);
}

/**
 * @brief Get the configured number of standard deviations for outlier gating.
 * @return Number of sigmas.
 */
double CobConverter::getNumStandardDeviations() const { return this->algorithm.getNumStandardDeviations(); }

/**
 * @brief Set an explicit standard deviation for the expected COB error.
 * @param num Standard deviation (> 0).
 * @note When set, outlier detection will use this fixed value instead of deriving one.
 */
void CobConverter::setStandardDeviation(const double num) {
    assert(num > 0.0);
    this->algorithm.setStandardDeviation(num);
}

/**
 * @brief Get the explicitly specified standard deviation (if set).
 * @return Standard deviation value.
 */
double CobConverter::getStandardDeviation() const { return this->algorithm.getStandardDeviation(); }

/**
 * @brief Determine whether a standard deviation has been explicitly specified.
 * @return True if specified, false otherwise.
 */
bool CobConverter::isStandardDeviationSpecified() const { return this->algorithm.isStandardDeviationSpecified(); }

/**
 * @brief Enable COB outlier detection.
 */
void CobConverter::enableOutlierDetection() { this->algorithm.enableOutlierDetection(); }

/**
 * @brief Disable COB outlier detection.
 */
void CobConverter::disableOutlierDetection() { this->algorithm.disableOutlierDetection(); }

/**
 * @brief Check whether COB outlier detection is enabled.
 * @return True if enabled, false otherwise.
 */
bool CobConverter::isOutlierDetectionEnabled() const { return this->algorithm.isOutlierDetectionEnabled(); }
