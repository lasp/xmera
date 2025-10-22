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

#ifndef MRP_STEERING_H
#define MRP_STEERING_H

#include <architecture/_GeneralModuleFiles/sys_model.h>
#include <architecture/messaging/messaging.h>
#include <architecture/msgPayloadDef/AttGuidMsgPayload.h>
#include <architecture/msgPayloadDef/RateCmdMsgPayload.h>
#include "mrpSteeringAlgorithm.h"
#include <stdint.h>

/*! @brief Data structure for the MRP feedback attitude control routine. */
class MrpSteering : public SysModel {
   public:
    MrpSteering() = default;
    ~MrpSteering() final = default;

    void reset(uint64_t callTime) override;
    void updateState(uint64_t callTime) override;

    void setK1(const double gain);
    double getK1() const;
    void setK3(const double gain);
    double getK3() const;
    void setOmegaMax(const double omega);
    double getOmegaMax() const;
    void setIgnoreFeedforward(const bool ignore);
    bool getIgnoreFeedforward() const;

    Message<RateCmdMsgPayload> rateCmdOutMsg;  //!< rate command output message
    ReadFunctor<AttGuidMsgPayload> guidInMsg;  //!< attitude guidance input message

   private:
    MrpSteeringAlgorithm algorithm{};
};

#endif
