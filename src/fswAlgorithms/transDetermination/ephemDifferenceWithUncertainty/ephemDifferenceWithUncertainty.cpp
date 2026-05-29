// SPDX-License-Identifier: ISC
// Copyright (c) 2024, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

#include "ephemDifferenceWithUncertainty.h"

/*! This method resets the module state to its initialized default.
 @return void
 @param currentSimNanos The clock time at which the function was called (nanoseconds)
 */
void EphemDifferenceWithUncertainty::reset(uint64_t currentSimNanos) {
    if (!this->ephemBaseInMsg.isLinked()) {
        throw std::invalid_argument("EphemDifferenceWithUncertainty.ephemBaseInMsg wasn't connected.");
    }
    if (!this->ephemSecondaryInMsg.isLinked()) {
        throw std::invalid_argument("EphemDifferenceWithUncertainty.ephemSecondaryInMsg wasn't connected.");
    }
}

/*! During an update, this module computes the difference between two ephemeris messages.
 @return void
 @param currentSimNanos The clock time at which the function was called (nanoseconds)
 */
void EphemDifferenceWithUncertainty::updateState(uint64_t currentSimNanos) {
    EphemerisMsgPayload ephemBaseInBuffer = this->ephemBaseInMsg();
    EphemerisMsgPayload ephemSecondaryInBuffer = this->ephemSecondaryInMsg();

    auto [navTransOutMsgBuffer, filterOutMsgBuffer] =
        this->algorithm.updateState(ephemBaseInBuffer, ephemSecondaryInBuffer);

    this->navTransOutMsg.write(navTransOutMsgBuffer, this->moduleID, currentSimNanos);
    this->filterOutMsg.write(filterOutMsgBuffer, this->moduleID, currentSimNanos);
}

/*! Set the state covariance of the base celestial object (e.g. asteroid)
    @param Eigen::MatrixXd covariance
    @return void
    */
void EphemDifferenceWithUncertainty::setCovarianceBase(const Eigen::MatrixXd stateCovariance) {
    this->algorithm.setCovarianceBase(stateCovariance);
}

/*! Get the state covariance of the base celestial object (e.g. asteroid)
    @return Eigen::MatrixXd covariance
    */
Eigen::MatrixXd EphemDifferenceWithUncertainty::getCovarianceBase() const {
    return this->algorithm.getCovarianceBase();
}

/*! Set the state covariance of the secondary celestial object (e.g. spacecraft)
    @param Eigen::MatrixXd covariance
    @return void
    */
void EphemDifferenceWithUncertainty::setCovarianceSecondary(const Eigen::MatrixXd stateCovariance) {
    this->algorithm.setCovarianceSecondary(stateCovariance);
}

/*! Get the state covariance of the secondary celestial object (e.g. spacecraft)
    @return Eigen::MatrixXd covariance
    */
Eigen::MatrixXd EphemDifferenceWithUncertainty::getCovarianceSecondary() const {
    return this->algorithm.getCovarianceSecondary();
}
