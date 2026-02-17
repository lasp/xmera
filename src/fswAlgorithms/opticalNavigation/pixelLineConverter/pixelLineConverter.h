// SPDX-License-Identifier: ISC
// Copyright (c) 2016, Autonomous Vehicle System Lab, University of Colorado at Boulder
// Copyright (c) 2025, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

#ifndef _PIXEL_LINE_CONVERTER_H_
#define _PIXEL_LINE_CONVERTER_H_

#include <architecture/_GeneralModuleFiles/sys_model.h>
#include <architecture/messaging/messaging.h>
#include <architecture/msgPayloadDef/CameraConfigMsgPayload.h>
#include <architecture/msgPayloadDef/NavAttMsgPayload.h>
#include <architecture/msgPayloadDef/OpNavCirclesMsgPayload.h>
#include <architecture/msgPayloadDef/OpNavMsgPayload.h>

#include <architecture/utilities/astroConstants.h>
#include <architecture/utilities/bskLogging.h>
#include <architecture/utilities/linearAlgebra.h>
#include <architecture/utilities/macroDefinitions.h>
#include <architecture/utilities/rigidBodyKinematics.h>

/*! @brief The configuration structure for the pixelLine Converter module.*/
class PixelLineConverter : public SysModel {
   public:
    void reset(uint64_t callTime) override;
    void updateState(uint64_t callTime) override;

    Message<OpNavMsgPayload> opNavOutMsg;                   //!< [-] output navigation message for relative position
    ReadFunctor<CameraConfigMsgPayload> cameraConfigInMsg;  //!< camera config input message
    ReadFunctor<NavAttMsgPayload> attInMsg;                 //!< attitude input message
    ReadFunctor<OpNavCirclesMsgPayload> circlesInMsg;       //!< circles input message

    int32_t planetTarget;  //!< The planet targeted (None = 0, Earth = 1, Mars = 2, Jupiter = 3 are allowed)

    BSKLogger bskLogger = {};  //!< BSK Logging
};

#endif
