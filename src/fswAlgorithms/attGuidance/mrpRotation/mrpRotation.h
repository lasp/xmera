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

#ifndef MRP_ROTATION_H
#define MRP_ROTATION_H

#include <stdint.h>

#include <xmera/sys_model.h>
#include "architecture/messaging/messaging.h"
#include "architecture/msgPayloadDef/AttRefMsgPayload.h"
#include "architecture/msgPayloadDef/AttStateMsgPayload.h"
#include "fswAlgorithms/attGuidance/mrpRotation/mrpRotationAlgorithm.h"
#include <Eigen/Core>

/*! @brief MRP Rotation class */
class MrpRotation : public SysModel {
   public:
    MrpRotation() = default;
    ~MrpRotation() final = default;

    void reset(uint64_t callTime) override;
    void updateState(uint64_t callTime) override;

    void setSigmaRR0(const Eigen::Vector3d &sigma);
    const Eigen::Vector3d &getSigmaRR0() const;
    void setOmegaRR0(const Eigen::Vector3d &omega);
    const Eigen::Vector3d &getOmegaRR0() const;

    Message<AttRefMsgPayload> attRefOutMsg;           //!< output message containing the Reference
    ReadFunctor<AttRefMsgPayload> attRefInMsg;        //!< guidance reference input message
    ReadFunctor<AttStateMsgPayload> desiredAttInMsg;  //!< incoming message containing the desired attitude set

   private:
    MrpRotationAlgorithm algorithm{};
};

#endif
