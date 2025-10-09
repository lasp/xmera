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

#ifndef THR_FIRING_REMAINDER_ALGORITHM
#define THR_FIRING_REMAINDER_ALGORITHM

#include "architecture/msgPayloadDef/THRArrayCmdForceMsgPayload.h"
#include "architecture/msgPayloadDef/THRArrayConfigMsgPayload.h"
#include "architecture/msgPayloadDef/THRArrayOnTimeCmdMsgPayload.h"

#include <array>
#include <stdint.h>

class ThrFiringRemainderAlgorithm {
   public:
    void reset(uint64_t callTime, const THRArrayConfigMsgPayload& thrConfigInMsgPayload);
    THRArrayOnTimeCmdMsgPayload update(uint64_t callTime, THRArrayCmdForceMsgPayload& thrForceInMsgPayload);

    void setThrMinFireTime(double thrMinFireTime);
    double getThrMinFireTime() const;

    void setBaseThrustState(int baseThrustState);
    int getBaseThrustState() const;

    void setDefaultControlPeriod(double defaultControlPeriod);
    double getDefaultControlPeriod() const;

   private:
    uint64_t prevCallTime{};                          //!< callTime from previous function call
    std::array<double, MAX_EFF_CNT> pulseRemainder{}; //!< [-] Unimplemented thrust pulses (number of minimum pulses)
    double thrMinFireTime{};                          //!< [s] Minimum fire time
    int numThrusters{};                               //!< [-] The number of thrusters available on vehicle
    std::array<double, MAX_EFF_CNT> maxThrust{};      //!< [N] Max thrust
    int baseThrustState{};                            //!< [-] Indicates on-pulsing (0) or off-pulsing (1)
    double defaultControlPeriod{};                    //!< [s] Default control period used for first call //Setter and Getter
};

#endif
