#ifndef MRP_STEERING_ALGORITHM_H
#define MRP_STEERING_ALGORITHM_H

#include <architecture/msgPayloadDef/AttGuidMsgPayload.h>
#include <architecture/msgPayloadDef/RateCmdMsgPayload.h>

/*! @brief Data structure for the MRP feedback attitude control routine. */
class MrpSteeringAlgorithm {
   public:
    RateCmdMsgPayload update(AttGuidMsgPayload& guidInMsg) const;

    void setK1(const double gain);
    double getK1() const;
    void setK3(const double gain);
    double getK3() const;
    void setOmegaMax(const double omega);
    double getOmegaMax() const;
    void setIgnoreFeedforward(const bool ignore);
    bool getIgnoreFeedforward() const;

   private:
    double K1{};                        //!< [rad/sec] Proportional gain applied to MRP errors
    double K3{};                        //!< [rad/sec] Cubic gain applied to MRP error in steering saturation function
    double omegaMax{};                  //!< [rad/sec] Maximum rate command of steering control
    bool ignoreOuterLoopFeedforward{};  //!< [] Boolean flag indicating if outer feedforward term should be included
};

#endif
