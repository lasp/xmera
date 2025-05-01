/*
 ISC License

 Copyright (c) 2016, Autonomous Vehicle Systems Lab, University of Colorado at Boulder

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

#ifndef BASILISK_ATT_TRACKING_ERROR_H
#define BASILISK_ATT_TRACKING_ERROR_H

#include <stdint.h>

#include <Eigen/Dense>

#include "architecture/_GeneralModuleFiles/sys_model.h"
#include "architecture/messaging/messaging.h"
#include "architecture/msgPayloadDefC/AttGuidMsgPayload.h"
#include "architecture/msgPayloadDefC/AttRefMsgPayload.h"
#include "architecture/msgPayloadDefC/NavAttMsgPayload.h"
#include "architecture/utilities/bskLogging.h"
#include "fswAlgorithms/attGuidance/attTrackingError/attTrackingErrorAlgorithm.h"

/*!@brief Data structure for module to compute the attitude tracking error between the spacecraft attitude and the
 * reference.
 */
class AttTrackingError : public SysModel {
   public:
    AttTrackingError() = default;   //!< Constructor
    ~AttTrackingError() = default;  //!< Destructor
    void reset(uint64_t callTime) override;
    void updateState(uint64_t callTime) override;
    void setSigma_R0R(const Eigen::Vector3d &sigma_R0R);
    const Eigen::Vector3d &getSigma_R0R() const;

    Message<AttGuidMsgPayload> attGuidOutMsg;   //!< Output attitude guidance message
    ReadFunctor<NavAttMsgPayload> attNavInMsg;  //!< Input msg measured attitude
    ReadFunctor<AttRefMsgPayload> attRefInMsg;  //!< Input msg of reference attitude
    BSKLogger bskLogger = {};                   //!< BSK Logging

   private:
    AttTrackingErrorAlgorithm algorithm;
};

#endif
