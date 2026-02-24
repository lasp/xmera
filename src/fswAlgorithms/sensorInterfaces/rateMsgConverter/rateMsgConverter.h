// SPDX-License-Identifier: ISC
// Copyright (c) 2016, Autonomous Vehicle System Lab, University of Colorado at Boulder
// Copyright (c) 2025, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

#ifndef _RATE_IMU_TO_NAV_CONVERTER_H_
#define _RATE_IMU_TO_NAV_CONVERTER_H_

#include <architecture/_GeneralModuleFiles/sys_model.h>
#include <architecture/messaging/messaging.h>
#include <architecture/msgPayloadDef/IMUSensorBodyMsgPayload.h>
#include <architecture/msgPayloadDef/NavAttMsgPayload.h>
#include <stdint.h>

#include "architecture/utilities/bskLogging.h"

/*! @brief Top level structure for the sub-module routines. */
class RateMsgConverter : public SysModel {
   public:
    void updateState(uint64_t callTime) override;
    void reset(uint64_t callTime) override;

    /* declare module IO interfaces */
    Message<NavAttMsgPayload> navRateOutMsg;            //!< attitude output message*/
    ReadFunctor<IMUSensorBodyMsgPayload> imuRateInMsg;  //!< attitude Input message*/

    BSKLogger bskLogger = {};  //!< BSK Logging
};

#endif
