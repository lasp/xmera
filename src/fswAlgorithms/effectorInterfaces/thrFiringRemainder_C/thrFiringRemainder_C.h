// SPDX-License-Identifier: ISC
// Copyright (c) 2016, Autonomous Vehicle System Lab, University of Colorado at Boulder

#ifndef THR_FIRING_REMAINDER_C_
#define THR_FIRING_REMAINDER_C_

#include "thrFiringRemainderAlgorithm_C.h"

#include <architecture/_GeneralModuleFiles/sys_model.h>
#include <architecture/messaging/messaging.h>
#include <architecture/msgPayloadDef/THRArrayCmdForceMsgPayload.h>
#include <architecture/msgPayloadDef/THRArrayConfigMsgPayload.h>
#include <architecture/msgPayloadDef/THRArrayOnTimeCmdMsgPayload.h>
#include <architecture/utilities/bskLogging.h>

#include <stdint.h>

/*! @brief Top level structure for the sub-module routines. */
class ThrFiringRemainder_C : public SysModel {
   public:
    void reset(uint64_t callTime) override;
    void updateState(uint64_t callTime) override;

    void setThrMinFireTime(double thrMinFireTime);
    double getThrMinFireTime() const;

    void setBaseThrustState(int baseThrustState);
    int getBaseThrustState() const;

    void setDefaultControlPeriod(double defaultControlPeriod);
    double getDefaultControlPeriod() const;

    /* declare module IO interfaces */
    ReadFunctor<THRArrayCmdForceMsgPayload> thrForceInMsg;  //!< The name of the Input message
    Message<THRArrayOnTimeCmdMsgPayload> onTimeOutMsg;      //!< The name of the output message, onTimeOutMsg
    ReadFunctor<THRArrayConfigMsgPayload> thrConfInMsg;     //!< The name of the thruster cluster Input message
    BSKLogger bskLogger = {};                               //!< BSK Logging

   private:
    ThrFiringRemainderInternalState algorithmState{};
};

#endif
