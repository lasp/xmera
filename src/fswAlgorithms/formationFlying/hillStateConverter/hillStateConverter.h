// SPDX-License-Identifier: ISC
// Copyright (c) 2016, Autonomous Vehicle System Lab, University of Colorado at Boulder
// Copyright (c) 2025, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

#ifndef _HILL_STATE_CONVERTER_H_
#define _HILL_STATE_CONVERTER_H_

//  Standard lib imports
#include <stdint.h>

//  Support imports
#include <architecture/utilities/bskLogging.h>

//  Message type imports
#include <architecture/_GeneralModuleFiles/sys_model.h>
#include <architecture/messaging/messaging.h>
#include <architecture/msgPayloadDef/HillRelStateMsgPayload.h>
#include <architecture/msgPayloadDef/NavTransMsgPayload.h>

/*! @brief Top level structure for the sub-module routines. */
class HillStateConverter : public SysModel {
   public:
    void reset(uint64_t callTime) override;
    void updateState(uint64_t callTime) override;

    /* declare module IO interfaces */
    Message<HillRelStateMsgPayload>
        hillStateOutMsg;  //!< Output message containing relative state of deputy to chief in chief hill coordinates
    ReadFunctor<NavTransMsgPayload>
        chiefStateInMsg;  //!< Input message containing chief inertial translational state estimate
    ReadFunctor<NavTransMsgPayload>
        depStateInMsg;  //!< Input message containing deputy inertial translational state estimate

    BSKLogger bskLogger = {};  //!< BSK Logging
};

#endif
