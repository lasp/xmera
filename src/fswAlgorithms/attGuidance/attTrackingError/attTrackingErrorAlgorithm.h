// SPDX-License-Identifier: ISC
// Copyright (c) 2025, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

#ifndef _ATT_TRACKING_ERROR_ALGORITHM_H
#define _ATT_TRACKING_ERROR_ALGORITHM_H

#include <Eigen/Core>

#include <architecture/msgPayloadDef/AttGuidMsgPayload.h>
#include <architecture/msgPayloadDef/AttRefMsgPayload.h>
#include <architecture/msgPayloadDef/NavAttMsgPayload.h>
#include <fswAlgorithms/fswUtilities/fswDefinitions.h>

class AttTrackingErrorAlgorithm {
   public:
    void reset(uint64_t callTime);  //!< Method for algorithm reset
    AttGuidMsgPayload update(uint64_t callTime,
                             AttRefMsgPayload& attRefInMsg,
                             NavAttMsgPayload& attNavInMsg);  //!< Algorithm update method
    void setSigma_R0R(const Eigen::Vector3d& sigma_R0R);      //!< Setter for sigma_R0R variable
    const Eigen::Vector3d& getSigma_R0R() const;              //!< Getter for sigma_R0R variable

   private:
    Eigen::Vector3d sigma_R0R{};  //!< MRP from corrected reference frame to original reference frame R0.
};

#endif
