// SPDX-License-Identifier: ISC
// Copyright (c) 2016, Autonomous Vehicle System Lab, University of Colorado at Boulder
// Copyright (c) 2025, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

#ifndef _RW_CONFIG_DATA_H_
#define _RW_CONFIG_DATA_H_

#include <architecture/_GeneralModuleFiles/sys_model.h>
#include <architecture/messaging/messaging.h>
#include <architecture/msgPayloadDef/RWArrayConfigMsgPayload.h>
#include <architecture/msgPayloadDef/RWConstellationMsgPayload.h>

#include <architecture/utilities/bskLogging.h>
#include <stdint.h>

/*! @brief Top level structure for the sub-module routines. */
class RwConfig : public SysModel {
   public:
    void reset(uint64_t callTime) override;
    void updateState(uint64_t callTime) override { /* Not Implemented */ }

    /* declare module IO interfaces */
    ReadFunctor<RWConstellationMsgPayload> rwConstellationInMsg; /*!< RW array input message */
    Message<RWArrayConfigMsgPayload> rwParamsOutMsg;             /*!< RW array output message */

    BSKLogger bskLogger = {};  //!< BSK Logging
};

#endif
