// SPDX-License-Identifier: ISC
// Copyright (c) 2016, Autonomous Vehicle System Lab, University of Colorado at Boulder

#ifndef THR_FIRING_REMAINDER_ALGORITHM
#define THR_FIRING_REMAINDER_ALGORITHM

#include "thrFiringRemainderTypes.h"

#include "architecture/msgPayloadDef/THRArrayCmdForceMsgPayload.h"
#include "architecture/msgPayloadDef/THRArrayConfigMsgPayload.h"
#include "architecture/msgPayloadDef/THRArrayOnTimeCmdMsgPayload.h"

#include <stdint.h>
#include <array>

class ThrFiringRemainderAlgorithm {
   public:
    void reset(const THRArrayConfigMsgPayload& thrConfigInMsgPayload);
    THRArrayOnTimeCmdMsgPayload update(uint64_t callTime, THRArrayCmdForceMsgPayload thrForceInMsgPayload);

    void setThrMinFireTime(double thrMinFireTime);
    double getThrMinFireTime() const;

    void setThrustPulsingRegime(ThrustPulsingRegime thrustPulsingRegime);
    ThrustPulsingRegime getThrustPulsingRegime() const;

    void setDefaultControlPeriod(double defaultControlPeriod);
    double getDefaultControlPeriod() const;

   private:
    uint64_t prevCallTime{};                           //!< callTime from previous function call
    std::array<double, MAX_EFF_CNT> pulseRemainder{};  //!< [-] Unimplemented thrust pulses (number of minimum pulses)
    double thrMinFireTime{};                           //!< [s] Minimum fire time
    int numThrusters{};                                //!< [-] The number of thrusters available on vehicle
    std::array<double, MAX_EFF_CNT> maxThrust{};       //!< [N] Max thrust
    ThrustPulsingRegime thrustPulsingRegime{};         //!< [-] Indicates on-pulsing or off-pulsing
    double defaultControlPeriod{};  //!< [s] Default control period used for first call //Setter and Getter
};

#endif
