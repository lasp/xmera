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

#ifndef _MRP_STEERING_ALGORITHM_H_
#define _MRP_STEERING_ALGORITHM_H_

#include "architecture/msgPayloadDefC/AttGuidMsgPayload.h"
#include "architecture/msgPayloadDefC/RateCmdMsgPayload.h"
#include <Eigen/Dense>
#include <cstdint>

class MrpSteeringAlgorithm {
  public:
    MrpSteeringAlgorithm() = default;
    ~MrpSteeringAlgorithm() = default;

    RateCmdMsgPayload update(uint64_t callTime, AttGuidMsgPayload guidCmd);

    double getK1() const;
    double getK3() const;
    double getOmegaMax() const;
    bool getIgnoreOuterLoopFeedforward() const;
    void setK1(double K1);
    void setK3(double K3);
    void setOmegaMax(double omegaMax);
    void setIgnoreOuterLoopFeedforward(bool ignore);

  private:
    double K1{};
    double K3{};
    double omega_max{};
    bool ignoreOuterLoopFeedforward{false};
};

#endif
