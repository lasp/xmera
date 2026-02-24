// SPDX-License-Identifier: ISC
// Copyright (c) 2025, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

#include "timeClosestApproach.h"

/*! Module constructor */
TimeClosestApproach::TimeClosestApproach() = default;

/*! Module destructor */
TimeClosestApproach::~TimeClosestApproach() = default;

/*! Read input messages.
 @return void
 */
void TimeClosestApproach::readMessages() {
    /*! - Read the input messages */
    auto filterStatePayload = this->filterInMsg();
    auto navFilterMsgPayload = this->navFilterMsg();

    this->numberOfStates = filterStatePayload.numberOfStates;
    this->r_BN_N = cArrayToEigenVector(navFilterMsgPayload.r_BN_N);
    this->v_BN_N = cArrayToEigenVector(navFilterMsgPayload.v_BN_N);
    this->filterCovariance = cArrayToEigenMatrixX(filterStatePayload.covar, this->numberOfStates, this->numberOfStates);
}

/*! Write output messages.
* @return void
* @param tCA time Closest Approach
* @param sigmaTca standard deviation of Time closest approach
* @param currentSimNanos Current sim nano

*/
void TimeClosestApproach::writeMessages(const double tCA, const double sigmaTca, const uint64_t currentSimNanos) {
    /*! create and zero the output message */
    TimeClosestApproachMsgPayload tcaMsgBuffer{};
    tcaMsgBuffer.timeClosestApproach = tCA;
    tcaMsgBuffer.standardDeviation = sigmaTca;

    /*! Write the output messages */
    this->tcaOutMsg.write(&tcaMsgBuffer, this->moduleID, currentSimNanos);
}

/*! Compute flyby geometry variables f0 & gamma0.
 @return void
 */
void TimeClosestApproach::computeGeometry() {
    /*! - compute velocity/radius ratio at time of read */
    this->ratio = this->v_BN_N.norm() / this->r_BN_N.norm();

    // compute an angle at the time of read
    Eigen::Vector3d r_BN_N_hat = this->r_BN_N.normalized();
    Eigen::Vector3d v_BN_N_hat = this->v_BN_N.normalized();

    double product = -r_BN_N_hat.dot(v_BN_N_hat);
    product = std::max(-1.0, std::min(1.0, product));
    const double theta = std::acos(product);

    // compute flight path angle at the time of read
    this->flightPathAngle = theta - M_PI / 2.0;
}

/*! Compute time of closest approach.
 * @return double time of closest approach in sec
 */
double TimeClosestApproach::computeTca() const { return -std::sin(this->flightPathAngle) / this->ratio; }

/*! Compute standard deviation of time closest approach.
 @return double standard deviation of time closest approach.
 */
double TimeClosestApproach::computeTcaStandardDeviation() const {
    Eigen::Vector3d r_BN_N_hat = this->r_BN_N.normalized();
    Eigen::Vector3d v_BN_N_hat = this->v_BN_N.normalized();

    // Calculate covariance_map_to_tca
    Eigen::VectorXd covariance_map_to_tca(this->numberOfStates);

    covariance_map_to_tca.head(3) = v_BN_N_hat / r_BN_N.norm();
    if (this->numberOfStates == 6) {
        covariance_map_to_tca.tail(3) =
            1.0 / v_BN_N.norm() * (r_BN_N_hat - std::sin(this->flightPathAngle) * v_BN_N_hat);
    }
    const double mappedCovariance = covariance_map_to_tca.transpose() * this->filterCovariance * covariance_map_to_tca;
    const double tCA_covariance = (1.0 / std::pow(this->ratio, 2)) * mappedCovariance;

    return std::sqrt(tCA_covariance);
}

/*! This method is the main carrier for the time of closest approach calculation
 @return void
 @param currentSimNanos The current simulation time for system
 */
void TimeClosestApproach::updateState(const uint64_t currentSimNanos) {
    this->readMessages();
    this->computeGeometry();
    const double tCA = this->computeTca();
    const double sigmaTca = this->computeTcaStandardDeviation();
    this->writeMessages(tCA, sigmaTca, currentSimNanos);
}
