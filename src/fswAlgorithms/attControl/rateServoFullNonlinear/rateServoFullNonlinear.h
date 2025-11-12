#ifndef RATE_SERVO_FULL_NONLINEAR_H
#define RATE_SERVO_FULL_NONLINEAR_H

#include <stdint.h>

#include <architecture/_GeneralModuleFiles/sys_model.h>
#include <architecture/messaging/messaging.h>
#include <architecture/msgPayloadDef/AttGuidMsgPayload.h>
#include <architecture/msgPayloadDef/CmdTorqueBodyMsgPayload.h>
#include <architecture/msgPayloadDef/RWArrayConfigMsgPayload.h>
#include <architecture/msgPayloadDef/RWAvailabilityMsgPayload.h>
#include <architecture/msgPayloadDef/RWSpeedMsgPayload.h>
#include <architecture/msgPayloadDef/RateCmdMsgPayload.h>
#include <architecture/msgPayloadDef/VehicleConfigMsgPayload.h>
#include "rateServoFullNonlinearAlgorithm.h"

#include <Eigen/Core>

/*! @brief The configuration structure for the rateServoFullNonlinear module.  */
class RateServoFullNonlinear : public SysModel {
   public:
    void reset(uint64_t callTime) override;
    void updateState(uint64_t callTime) override;

    void setP(const double gain);
    double getP() const;
    void setKi(const double gain);
    double getKi() const;
    void setIntegralLimit(const double limit);
    double getIntegralLimit() const;
    void setKnownTorquePntB_B(const Eigen::Vector3d& knownTorquePntB_B);
    Eigen::Vector3d getKnownTorquePntB_B() const;

    /* declare module IO interfaces */
    Message<CmdTorqueBodyMsgPayload> cmdTorqueOutMsg;     //!< commanded torque output message
    ReadFunctor<AttGuidMsgPayload> guidInMsg;             //!< attitude guidance input message
    ReadFunctor<VehicleConfigMsgPayload> vehConfigInMsg;  //!< vehicle configuration input message
    ReadFunctor<RWSpeedMsgPayload> rwSpeedsInMsg;         //!< (optional) RW speed input message
    ReadFunctor<RWAvailabilityMsgPayload> rwAvailInMsg;   //!< (optional) RW availability input message
    ReadFunctor<RWArrayConfigMsgPayload> rwParamsInMsg;   //!< (optional) RW configuration parameter input message
    ReadFunctor<RateCmdMsgPayload> rateSteeringInMsg;     //!< commanded rate input message

   private:
    RateServoFullNonlinearAlgorithm algorithm{};
    uint32_t numRW{};  //!< number of reaction wheels
};

#endif
