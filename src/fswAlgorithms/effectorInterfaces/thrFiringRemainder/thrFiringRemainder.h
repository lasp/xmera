/*
 ISC License

 Copyright (c) 2016, Autonomous Vehicle Systems Lab, University of Colorado at Boulder

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

#ifndef _THR_FIRING_REMAINDER_
#define _THR_FIRING_REMAINDER_

#include <stdint.h>

#include "architecture/_GeneralModuleFiles/sys_model.h"
#include "architecture/messaging/messaging.h"
#include "architecture/msgPayloadDefC/THRArrayCmdForceMsgPayload.h"
#include "architecture/msgPayloadDefC/THRArrayConfigMsgPayload.h"
#include "architecture/msgPayloadDefC/THRArrayOnTimeCmdMsgPayload.h"
#include "architecture/utilities/bskLogging.h"
#include "fswAlgorithms/effectorInterfaces/thrFiringRemainder/thrFiringRemainderAlgorithm.h"

/*! @brief Top level structure for the sub-module routines. */
class ThrFiringRemainder : public SysModel {
   public:
    ThrFiringRemainder() = default;   //!< Constructor
    ~ThrFiringRemainder() = default;  //!< Destructor
    void reset(uint64_t callTime) override;
    void updateState(uint64_t callTime) override;

    void setThrMinFireTime(const double thrMinFireTime);  //!< Setter for thrMinFireTime variable
    const double& getThrMinFireTime() const;              //!< Getter for thrMinFireTime variable

    void setBaseThrustState(const int baseThrustState);  //!< Setter for baseThrustState variable
    const int& getBaseThrustState() const;               //!< Getter for baseThrustState variable

    void setDefaultControlPeriod(const double defaultControlPeriod);  //!< Setter for defaultControlPeriod variable
    const double& getDefaultControlPeriod() const;                    //!< Getter for defaultControlPeriod variable

    /* declare module IO interfaces */
    ReadFunctor<THRArrayCmdForceMsgPayload> thrForceInMsg;  //!< The name of the Input message
    Message<THRArrayOnTimeCmdMsgPayload> onTimeOutMsg;      //!< The name of the output message, onTimeOutMsg
    ReadFunctor<THRArrayConfigMsgPayload> thrConfInMsg;     //!< The name of the thruster cluster Input message
    BSKLogger bskLogger = {};                               //!< BSK Logging

   private:
    ThrFiringRemainderAlgorithm algorithm;
};

#endif
