/*
 ISC License

 Copyright (c) 2024, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

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

#ifndef BASILISK_RATE_DAMP_H
#define BASILISK_RATE_DAMP_H

#include <xmera/sys_model.h>
#include "architecture/messaging/messaging.h"
#include "architecture/msgPayloadDef/CmdTorqueBodyMsgPayload.h"
#include "architecture/msgPayloadDef/NavAttMsgPayload.h"
#include <xmera/bskLogging.h>
#include "fswAlgorithms/attControl/rateDamp/rateDampAlgorithm.h"

/*! @brief Rate damp class */
class RateDamp : public SysModel {
   public:
    RateDamp() = default;                                 //!< Constructor
    ~RateDamp() = default;                                //!< Destructor
    void reset(uint64_t currentSimNanos) override;        //!< Reset method
    void updateState(uint64_t currentSimNanos) override;  //!< Update method
    void setRateGain(double p);                           //!< Setter method for rate feedback gain
    double getRateGain() const;                           //!< Getter method for rate feedback gain

    ReadFunctor<NavAttMsgPayload> attNavInMsg;         //!< Navigation input message
    Message<CmdTorqueBodyMsgPayload> cmdTorqueOutMsg;  //!< Command torque output message

   private:
    RateDampAlgorithm algorithm{};  //!< Algorithm for rateDamp control logic (BSK-agnostic)
};

#endif
