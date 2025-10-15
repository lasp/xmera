#ifndef RW_MOTOR_TORQUE_H
#define RW_MOTOR_TORQUE_H

#include <stdint.h>

#include <architecture/_GeneralModuleFiles/sys_model.h>
#include <architecture/messaging/messaging.h>
#include <architecture/msgPayloadDef/CmdTorqueBodyMsgPayload.h>
#include <architecture/msgPayloadDef/RWArrayConfigMsgPayload.h>
#include <architecture/msgPayloadDef/RWAvailabilityMsgPayload.h>
#include <architecture/msgPayloadDef/RwMotorTorqueMsgPayload.h>
#include "rwMotorTorqueAlgorithm.h"

#include <Eigen/Core>

/*! @brief Top level structure for the sub-module routines. */
class RwMotorTorque : public SysModel {
   public:
    RwMotorTorque() = default;
    ~RwMotorTorque() final = default;

    void reset(uint64_t callTime) override;
    void updateState(uint64_t callTime) override;

    void setControlAxes(const Eigen::Matrix3d& controlMappingMatrix);
    Eigen::Matrix3d getControlAxes() const;

    /* declare module IO interfaces */
    Message<RwMotorTorqueMsgPayload> rwMotorTorqueOutMsg;   //!< RW motor torque output message
    ReadFunctor<CmdTorqueBodyMsgPayload> vehControlInMsg;   //!<  vehicle control (Lr) Input message
    ReadFunctor<CmdTorqueBodyMsgPayload> vehControlIn2Msg;  //!<  optional vehicle control input message
    ReadFunctor<RWArrayConfigMsgPayload> rwParamsInMsg;     //!<  RW Array input message
    ReadFunctor<RWAvailabilityMsgPayload> rwAvailInMsg;     //!< optional RWs availability input message

   private:
    RwMotorTorqueAlgorithm algorithm{};
};

#endif
