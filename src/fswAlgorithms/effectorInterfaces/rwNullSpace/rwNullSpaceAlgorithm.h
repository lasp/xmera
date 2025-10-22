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

#ifndef RW_NULL_SPACE_ALGORITHM_H
#define RW_NULL_SPACE_ALGORITHM_H

#include <architecture/msgPayloadDef/RWConstellationMsgPayload.h>
#include <architecture/msgPayloadDef/RWSpeedMsgPayload.h>
#include <architecture/msgPayloadDef/RwMotorTorqueMsgPayload.h>

#include <Eigen/Core>

#include <stdint.h>
#include <stdlib.h>

/*! @brief The configuration structure for the rwNullSpace module.  */
class RwNullSpaceAlgorithm {
   public:
    void reset(RWConstellationMsgPayload& rwConfigInMsg);
    RwMotorTorqueMsgPayload update(RwMotorTorqueMsgPayload& controlRequest,
                                   RWSpeedMsgPayload& rwSpeeds,
                                   RWSpeedMsgPayload& rwDesiredSpeeds);

    void setOmegaGain(const double gain);
    double getOmegaGain() const;

   private:
    double omegaGain{};                                   //!< [-] The gain factor applied to the RW speeds
    Eigen::Matrix<double, RW_EFF_CNT, RW_EFF_CNT> tau{};  //!< [-] RW nullspace project matrix
    uint32_t numWheels{};                                 //!< [-] The number of reaction wheels we have
};

#endif
