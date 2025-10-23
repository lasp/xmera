/*
 ISC License

 Copyright (c) 2025, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

 Permission to use, copy, modify, and/or distribute this software for any
 purpose with or without fee is hereby granted, provided that the above
 copyright notice and this permission notice appear in all copies.

 THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
 WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
 MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
 ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
 WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
 ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
 OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.

 */

#ifndef XMERA_THRFIRINGSCHMITT_H
#define XMERA_THRFIRINGSCHMITT_H

#include <cstdint>

#include <architecture/_GeneralModuleFiles/sys_model.h>
#include <architecture/messaging/messaging.h>
#include <architecture/msgPayloadDef/THRArrayCmdForceMsgPayload.h>
#include <architecture/msgPayloadDef/THRArrayConfigMsgPayload.h>
#include <architecture/msgPayloadDef/THRArrayOnTimeCmdMsgPayload.h>
#include <architecture/utilities/bskLogging.h>
#include <architecture/utilities/macroDefinitions.h>
#include "thrFiringSchmittAlgorithm.h"

class ThrFiringSchmitt : public SysModel {
   public:
    ThrFiringSchmitt();

    void reset(uint64_t callTime) override;
    void updateState(uint64_t callTime) override;
    double getLevelOn() const;
    void setLevelOn(double level);
    double getLevelOff() const;
    void setLevelOff(double level);
    double getThrMinFireTime() const;
    void setThrMinFireTime(double time);
    uint32_t getBaseThrustState() const;
    void setBaseThrustState(uint32_t state);

    /* declare module IO interfaces */
    ReadFunctor<THRArrayCmdForceMsgPayload> thrForceInMsg;  //!< The name of the Input message
    Message<THRArrayOnTimeCmdMsgPayload> onTimeOutMsg;      //!< The name of the output message*, onTimeOutMsg
    ReadFunctor<THRArrayConfigMsgPayload> thrConfInMsg;     //!< The name of the thruster cluster Input message

    BSKLogger bskLogger = {};  //!< BSK Logging

   private:
    ThrFiringSchmittAlgorithm algorithm;
};

#endif  // XMERA_THRFIRINGSCHMITT_H
