// SPDX-License-Identifier: ISC
// Copyright (c) 2016, Autonomous Vehicle System Lab, University of Colorado at Boulder
// Copyright (c) 2025, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

#ifndef _HILL_POINT_
#define _HILL_POINT_

#include <stdint.h>

/* Required module input messages */
#include <architecture/_GeneralModuleFiles/sys_model.h>
#include <architecture/messaging/messaging.h>
#include <architecture/msgPayloadDef/AttRefMsgPayload.h>
#include <architecture/msgPayloadDef/EphemerisMsgPayload.h>
#include <architecture/msgPayloadDef/NavTransMsgPayload.h>

#include <architecture/utilities/bskLogging.h>

/*!@brief Data structure for module to compute the Hill-frame pointing navigation solution.
 */
class HillPoint : public SysModel {
   public:
    void reset(uint64_t callTime) override;
    void updateState(uint64_t callTime) override;

    /* declare module IO interfaces */
    Message<AttRefMsgPayload> attRefOutMsg;         //!<        The name of the output message
    ReadFunctor<NavTransMsgPayload> transNavInMsg;  //!<        The name of the incoming attitude command
    ReadFunctor<EphemerisMsgPayload> celBodyInMsg;  //!<        The name of the celestial body message

    int planetMsgIsLinked;  //!<        flag if the planet message is linked

    BSKLogger bskLogger = {};  //!< BSK Logging
};

#endif
