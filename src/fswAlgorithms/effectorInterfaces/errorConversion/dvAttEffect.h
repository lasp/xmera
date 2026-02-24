// SPDX-License-Identifier: ISC
// Copyright (c) 2016, Autonomous Vehicle System Lab, University of Colorado at Boulder
// Copyright (c) 2025, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

#ifndef _DV_ATT_EFFECT_H_
#define _DV_ATT_EFFECT_H_

#include "architecture/_GeneralModuleFiles/sys_model.h"
#include "architecture/messaging/messaging.h"
#include "architecture/msgPayloadDef/CmdTorqueBodyMsgPayload.h"
#include "architecture/msgPayloadDef/THRArrayOnTimeCmdMsgPayload.h"

#include "architecture/utilities/bskLogging.h"
#include "../_GeneralModuleFiles/thrustGroupData.h"
#include <stdint.h>
#include <stdlib.h>

#define MAX_NUM_THR_GROUPS 4

/*! @brief effective thruster pair structure */
typedef struct {
    double onTime;        /*!< s   The requested on time for this thruster*/
    uint32_t thrustIndex; /*!< -  The actual thruster index associated with on-time*/
} effPairs;

void computeSingleThrustBlock(ThrustGroupData* thrData,
                              uint64_t callTime,
                              CmdTorqueBodyMsgPayload* contrReq,
                              int64_t moduleID);

/*! @brief module configuration message */
class DvAttEffect : public SysModel {
   public:
    void reset(uint64_t callTime) override;
    void updateState(uint64_t callTime) override;
    ReadFunctor<CmdTorqueBodyMsgPayload> cmdTorqueBodyInMsg; /*!< - The name of the Input message*/

    uint32_t numThrGroups;                         /*!< - Count on the number of thrusters groups available*/
    ThrustGroupData thrGroups[MAX_NUM_THR_GROUPS]; /*!< - Thruster grouping container*/
    BSKLogger bskLogger = {};                      //!< BSK Logging
};

#endif
