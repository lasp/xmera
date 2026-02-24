// SPDX-License-Identifier: ISC
// Copyright (c) 2016, Autonomous Vehicle System Lab, University of Colorado at Boulder
// Copyright (c) 2025, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

#ifndef _FAULT_DETECTION_H_
#define _FAULT_DETECTION_H_

#include <architecture/_GeneralModuleFiles/sys_model.h>
#include <architecture/messaging/messaging.h>
#include <architecture/msgPayloadDef/CameraConfigMsgPayload.h>
#include <architecture/msgPayloadDef/NavAttMsgPayload.h>
#include <architecture/msgPayloadDef/OpNavMsgPayload.h>

#include <architecture/utilities/astroConstants.h>
#include <architecture/utilities/linearAlgebra.h>
#include <architecture/utilities/macroDefinitions.h>
#include <architecture/utilities/rigidBodyKinematics.h>

/*! @brief Module data structure */
class FaultDetection : public SysModel {
   public:
    void reset(uint64_t callTime) override;
    void updateState(uint64_t callTime) override;

    Message<OpNavMsgPayload> opNavOutMsg;                   //!< [-] output navigation message for relative position
    ReadFunctor<NavAttMsgPayload> attInMsg;                 //!< attitude input message
    ReadFunctor<OpNavMsgPayload> navMeasPrimaryInMsg;       //!< first measurement input message
    ReadFunctor<OpNavMsgPayload> navMeasSecondaryInMsg;     //!< second measurement input message
    ReadFunctor<CameraConfigMsgPayload> cameraConfigInMsg;  //!< camera config inut message

    int32_t planetTarget;  //!< The planet targeted (None = 0, Earth = 1, Mars = 2, Jupiter = 3 are allowed)
    double faultMode;      //!< What fault mode to go in: 0 is dissimilar (use the primary measurement and compare with
                           //!< secondary), 1 merges the measurements if they are both valid and similar.
    double sigmaFault;     //!< What is the sigma multiplication factor when comparing measurements

    // added for bsk
    BSKLogger bskLogger = {};  //!< BSK Logging
};

#endif
