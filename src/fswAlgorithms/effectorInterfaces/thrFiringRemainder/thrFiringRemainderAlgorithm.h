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

#ifndef _THR_FIRING_REMAINDER_ALGORITHM
#define _THR_FIRING_REMAINDER_ALGORITHM

#include "fswAlgorithms/fswUtilities/fswDefinitions.h"
#include <stdint.h>

#include "architecture/_GeneralModuleFiles/sys_model.h"
#include "architecture/messaging/messaging.h"
#include "architecture/msgPayloadDefC/THRArrayCmdForceMsgPayload.h"
#include "architecture/msgPayloadDefC/THRArrayConfigMsgPayload.h"
#include "architecture/msgPayloadDefC/THRArrayOnTimeCmdMsgPayload.h"

#include "architecture/utilities/bskLogging.h"
#include "architecture/utilities/macroDefinitions.h"

/*! @brief Top level structure for the sub-module routines. */
class ThrFiringRemainderAlgorithm {
   public:
    void reset(uint64_t callTime, THRArrayConfigMsgPayload thrConfigInMsgPayload);  //!< Method for algorithm reset
    THRArrayOnTimeCmdMsgPayload update(uint64_t callTime,
                                       THRArrayCmdForceMsgPayload& thrForceInMsg,
                                       THRArrayConfigMsgPayload& thrConfigInMsg);

    void setThrMinFireTime(const double thrMinFireTime);  //!< Setter for thrMinFireTime variable
    const double& getThrMinFireTime() const;              //!< Getter for thrMinFireTime variable

    void setBaseThrustState(const int baseThrustState);  //!< Setter for baseThrustState variable
    const int& getBaseThrustState() const;               //!< Getter for baseThrustState variable

    void setDefaultControlPeriod(const double defaultControlPeriod);  //!< Setter for defaultControlPeriod variable
    const double& getDefaultControlPeriod() const;                    //!< Getter for defaultControlPeriod variable

   private:
    uint64_t prevCallTime;               //!< callTime from previous function call
    double pulseRemainder[MAX_EFF_CNT];  //!< [-] Unimplemented thrust pulses (number of minimum pulses)
    double thrMinFireTime;               //!< [s] Minimum fire time
    int numThrusters;                    //!< [-] The number of thrusters available on vehicle
    double maxThrust[MAX_EFF_CNT];       //!< [N] Max thrust
    int baseThrustState;                 //!< [-] Indicates on-pulsing (0) or off-pulsing (1)
    double defaultControlPeriod;         //!< [s] Default control period used for first call //Setter and Getter
};

#endif
