// SPDX-License-Identifier: ISC
// Copyright (c) 2016, Autonomous Vehicle System Lab, University of Colorado at Boulder
// Copyright (c) 2025, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

#ifndef _DV_GUIDANCE_POINT_H_
#define _DV_GUIDANCE_POINT_H_

#include <architecture/_GeneralModuleFiles/sys_model.h>
#include <architecture/messaging/messaging.h>
#include <architecture/msgPayloadDef/AttRefMsgPayload.h>
#include <architecture/msgPayloadDef/DvBurnCmdMsgPayload.h>

#include <architecture/utilities/bskLogging.h>
#include <stdint.h>

/*! @brief Top level structure for the nominal delta-V guidance
 */
class DvGuidance : public SysModel {
   public:
    void reset(uint64_t callTime) override;
    void updateState(uint64_t callTime) override;
    Message<AttRefMsgPayload> attRefOutMsg;          //!< The name of the output message
    ReadFunctor<DvBurnCmdMsgPayload> burnDataInMsg;  //!< Input message that configures the vehicle burn

    BSKLogger bskLogger = {};  //!< BSK Logging
};

#endif
