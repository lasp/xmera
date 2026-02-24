// SPDX-License-Identifier: ISC
// Copyright (c) 2024, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

#ifndef EPHEM_DIFFERENCE_WITH_UNCERTAINTY_ALGORITHM_H
#define EPHEM_DIFFERENCE_WITH_UNCERTAINTY_ALGORITHM_H

#include <architecture/messaging/messaging.h>
#include <architecture/utilities/eigenSupport.h>
#include <stdint.h>
#include <Eigen/Dense>

#include <architecture/msgPayloadDef/EphemerisMsgPayload.h>
#include <architecture/msgPayloadDef/FilterMsgPayload.h>
#include <architecture/msgPayloadDef/NavTransMsgPayload.h>

/*! @brief This module computes the difference between two ephemeris messages, and outputs the relative states into a
 * navigation and filter message */
class EphemDifferenceWithUncertaintyAlgorithm {
   public:
    EphemDifferenceWithUncertaintyAlgorithm() = default;
    ~EphemDifferenceWithUncertaintyAlgorithm() = default;

    std::tuple<NavTransMsgPayload, FilterMsgPayload> updateState(EphemerisMsgPayload, EphemerisMsgPayload) const;
    void setCovarianceBase(const Eigen::MatrixXd stateCovariance);
    Eigen::MatrixXd getCovarianceBase() const;
    void setCovarianceSecondary(const Eigen::MatrixXd stateCovariance);
    Eigen::MatrixXd getCovarianceSecondary() const;

   private:
    Eigen::MatrixXd covarianceBase{Eigen::MatrixXd::Zero(6, 6)};
    Eigen::MatrixXd covarianceSecondary{Eigen::MatrixXd::Zero(6, 6)};
};

#endif
