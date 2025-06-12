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

#ifndef _MRP_STEERING_CONTROL_H_
#define _MRP_STEERING_CONTROL_H_

#include "architecture/_GeneralModuleFiles/sys_model.h"
#include "architecture/messaging/messaging.h"
#include "architecture/msgPayloadDefC/AttGuidMsgPayload.h"
#include "architecture/msgPayloadDefC/RateCmdMsgPayload.h"
#include "architecture/utilities/bskLogging.h"
#include "fswAlgorithms/attControl/mrpSteering/mrpSteeringAlgorithm.h"
#include <cstdint>



/*! @brief Data structure for the MRP feedback attitude control routine. */
class MrpSteering : public SysModel {
  public:
    MrpSteering() = default;
    ~MrpSteering() = default;
    void reset(uint64_t callTime) override;
    void updateState(uint64_t callTime) override;

    void setK1(double k1);
    double getK1() const;
    void setK3(double k3);
    double getK3() const;
    void setOmegaMax(double omegaMax);
    double getOmegaMax() const;
    void setIgnoreOuterLoopFeedforward(bool flag);
    bool getIgnoreOuterLoopFeedforward() const;

    Message<RateCmdMsgPayload> rateCmdOutMsg;  //!< rate command output message
    ReadFunctor<AttGuidMsgPayload> guidInMsg;   //!< attitude guidance input message

    BSKLogger bskLogger = {};  //!< BSK Logging

  private:
    MrpSteeringAlgorithm algorithm;
};

#endif
