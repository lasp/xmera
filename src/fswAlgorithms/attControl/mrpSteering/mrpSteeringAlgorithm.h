#ifndef BASILISK_MRP_STEERING_ALGORITHM_H
#define BASILISK_MRP_STEERING_ALGORITHM_H

#include <Eigen/Dense>
#include <cstdint>

#include "architecture/msgPayloadDefC/AttGuidMsgPayload.h"
#include "architecture/msgPayloadDefC/RateCmdMsgPayload.h"

class MrpSteeringAlgorithm {
  public:
    MrpSteeringAlgorithm() = default;
    ~MrpSteeringAlgorithm() = default;

    void reset(uint64_t callTime);
    RateCmdMsgPayload update(uint64_t callTime, AttGuidMsgPayload &guidCmd);

    void setK1(double k1);
    double getK1() const;
    void setK3(double k3);
    double getK3() const;
    void setOmegaMax(double omegaMax);
    double getOmegaMax() const;
    void setIgnoreOuterLoopFeedforward(bool flag);
    bool getIgnoreOuterLoopFeedforward() const;

  private:
    double K1{};
    double K3{};
    double omega_max{};
    bool ignoreOuterLoopFeedforward{false};
};

#endif
