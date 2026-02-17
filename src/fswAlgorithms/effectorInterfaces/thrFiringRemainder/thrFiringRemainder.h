// SPDX-License-Identifier: ISC
// Copyright (c) 2016, Autonomous Vehicle System Lab, University of Colorado at Boulder
// Copyright (c) 2025, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

#ifndef THR_FIRING_REMAINDER
#define THR_FIRING_REMAINDER

#include "thrFiringRemainderAlgorithm.h"

#include <architecture/_GeneralModuleFiles/sys_model.h>
#include <architecture/messaging/messaging.h>
#include <architecture/msgPayloadDef/THRArrayCmdForceMsgPayload.h>
#include <architecture/msgPayloadDef/THRArrayConfigMsgPayload.h>
#include <architecture/msgPayloadDef/THRArrayOnTimeCmdMsgPayload.h>
#include <architecture/utilities/bskLogging.h>

#include <stdint.h>

/*! @brief Top level structure for the sub-module routines. */
class ThrFiringRemainder : public SysModel {
   public:
    ThrFiringRemainder() = default;   //!< Constructor
    ~ThrFiringRemainder() = default;  //!< Destructor
    void reset(uint64_t callTime) override;
    void updateState(uint64_t callTime) override;

    void setThrMinFireTime(double thrMinFireTime);  //!< Setter for thrMinFireTime variable
    double getThrMinFireTime() const;               //!< Getter for thrMinFireTime variable

    void setThrustPulsingRegime(ThrustPulsingRegime thrustPulsingRegime);  //!< Setter for thrustPulsingRegime variable
    ThrustPulsingRegime getThrustPulsingRegime() const;                    //!< Getter for thrustPulsingRegime variable

    void setDefaultControlPeriod(double defaultControlPeriod);  //!< Setter for defaultControlPeriod variable
    double getDefaultControlPeriod() const;                     //!< Getter for defaultControlPeriod variable

    /* declare module IO interfaces */
    ReadFunctor<THRArrayCmdForceMsgPayload> thrForceInMsg;  //!< The name of the Input message
    Message<THRArrayOnTimeCmdMsgPayload> onTimeOutMsg;      //!< The name of the output message, onTimeOutMsg
    ReadFunctor<THRArrayConfigMsgPayload> thrConfInMsg;     //!< The name of the thruster cluster Input message
    BSKLogger bskLogger = {};                               //!< BSK Logging

   private:
    ThrFiringRemainderAlgorithm algorithm{};
};

#endif
