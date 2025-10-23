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

#ifndef RW_NULL_SPACE_H
#define RW_NULL_SPACE_H

#include <architecture/_GeneralModuleFiles/sys_model.h>
#include <architecture/messaging/messaging.h>
#include <architecture/msgPayloadDef/RWConstellationMsgPayload.h>
#include <architecture/msgPayloadDef/RWSpeedMsgPayload.h>
#include <architecture/msgPayloadDef/RwMotorTorqueMsgPayload.h>
#include "rwNullSpaceAlgorithm.h"

#include <stdint.h>

/*! @brief The configuration structure for the rwNullSpace module.  */
class RwNullSpace : public SysModel {
   public:
    RwNullSpace() = default;
    ~RwNullSpace() final = default;

    void reset(uint64_t callTime) override;
    void updateState(uint64_t callTime) override;

    void setOmegaGain(const double gain);
    double getOmegaGain() const;

    ReadFunctor<RwMotorTorqueMsgPayload> rwMotorTorqueInMsg;  //!< [-] The name of the Input message
    ReadFunctor<RWSpeedMsgPayload> rwSpeedsInMsg;             //!< [-] The name of the input RW speeds
    ReadFunctor<RWSpeedMsgPayload> rwDesiredSpeedsInMsg;      //!< [-] (optional) The name of the desired RW speeds
    ReadFunctor<RWConstellationMsgPayload> rwConfigInMsg;     //!< [-] The name of the RWA configuration message
    Message<RwMotorTorqueMsgPayload> rwMotorTorqueOutMsg;     //!< [-] The name of the output message

   private:
    RwNullSpaceAlgorithm algorithm{};
};

#endif
