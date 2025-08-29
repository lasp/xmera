/*
 ISC License

 Copyright (c) 2024, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

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

#ifndef EPHEM_DIFFERENCE_WITH_UNCERTAINTY_ALGORITHM_H
#define EPHEM_DIFFERENCE_WITH_UNCERTAINTY_ALGORITHM_H

#include "architecture/messaging/messaging.h"
#include "architecture/utilities/avsEigenSupport.h"
#include <stdint.h>
#include <Eigen/Dense>

#include "architecture/msgPayloadDef/EphemerisMsgPayload.h"
#include "architecture/msgPayloadDef/FilterMsgPayload.h"
#include "architecture/msgPayloadDef/NavTransMsgPayload.h"

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
