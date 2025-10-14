/*
 ISC License

 Copyright (c) 2025, University of Colorado at Boulder

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

#ifndef THRFIRINGREMAINDERALGORITHM_C_H
#define THRFIRINGREMAINDERALGORITHM_C_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

#include "architecture/msgPayloadDef/THRArrayCmdForceMsgPayload.h"
#include "architecture/msgPayloadDef/THRArrayConfigMsgPayload.h"
#include "architecture/msgPayloadDef/THRArrayOnTimeCmdMsgPayload.h"

#include "architecture/msgPayloadDef/definitions.h"

typedef struct {
    double pulseRemainder[MAX_EFF_CNT];  //!< [-] Unimplemented thrust pulses (number of minimum pulses)
    double thrMinFireTime;               //!< [s] Minimum fire time
    uint32_t numThrusters;               //!< [-] The number of thrusters available on vehicle
    double maxThrust[MAX_EFF_CNT];       //!< [N] Max thrust
    int baseThrustState;                 //!< [-] Indicates on-pulsing (0) or off-pulsing (1)
    double defaultControlPeriod;         //!< [s] Default control period used for first call
    uint64_t prevCallTime;               //!< callTime from previous function call
} ThrFiringRemainderInternalState;

void reset(ThrFiringRemainderInternalState *moduleState,
           uint64_t callTime,
           THRArrayConfigMsgPayload localThrusterData);

THRArrayOnTimeCmdMsgPayload updateState(ThrFiringRemainderInternalState *moduleState,
                                        uint64_t callTime,
                                        THRArrayCmdForceMsgPayload thrForceIn);

#ifdef __cplusplus
}
#endif

#endif //THRFIRINGREMAINDERALGORITHM_C_H