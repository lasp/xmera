// SPDX-License-Identifier: ISC
// Copyright (c) 2016, Autonomous Vehicle System Lab, University of Colorado at Boulder
// Copyright (c) 2025, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

#ifndef XMERA_ATT_TRACKING_ERROR_H
#define XMERA_ATT_TRACKING_ERROR_H

#include "attTrackingErrorAlgorithm.h"
#include <architecture/_GeneralModuleFiles/sys_model.h>
#include <architecture/messaging/messaging.h>
#include <architecture/msgPayloadDef/AttGuidMsgPayload.h>
#include <architecture/msgPayloadDef/AttRefMsgPayload.h>
#include <architecture/msgPayloadDef/NavAttMsgPayload.h>

#include <stdint.h>

#include <Eigen/Dense>

/*!@brief Data structure for module to compute the attitude tracking error between the spacecraft attitude and the
 * reference.
 */
class AttTrackingError : public SysModel {
public:
    AttTrackingError() = default;   //!< Constructor
    ~AttTrackingError() = default;  //!< Destructor
    void reset(uint64_t callTime) override;
    void updateState(uint64_t callTime) override;
    void setSigma_R0R(Eigen::Vector3d const &sigma_R0R);
    Eigen::Vector3d const &getSigma_R0R() const;

    Message<AttGuidMsgPayload> attGuidOutMsg;   //!< Output attitude guidance message
    ReadFunctor<NavAttMsgPayload> attNavInMsg;  //!< Input msg measured attitude
    ReadFunctor<AttRefMsgPayload> attRefInMsg;  //!< Input msg of reference attitude

private:
    AttTrackingErrorAlgorithm algorithm;
};

#endif
