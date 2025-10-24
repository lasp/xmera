#ifndef _RW_MOTOR_TORQUE_H_
#define _RW_MOTOR_TORQUE_H_

#include <stdint.h>

#include <architecture/_GeneralModuleFiles/sys_model.h>
#include <architecture/messaging/messaging.h>
#include <architecture/msgPayloadDef/CmdTorqueBodyMsgPayload.h>
#include <architecture/msgPayloadDef/RWArrayConfigMsgPayload.h>
#include <architecture/msgPayloadDef/RWAvailabilityMsgPayload.h>
#include <architecture/msgPayloadDef/RwMotorTorqueMsgPayload.h>

#include <Eigen/Core>

/*! @brief Top level structure for the sub-module routines. */
class RwMotorTorque : public SysModel {
   public:
    void reset(uint64_t callTime) override;
    void updateState(uint64_t callTime) override;

    Eigen::Matrix3d controlAxes_B{};  //!< [-] array of the control unit axes

    /* declare module IO interfaces */
    Message<RwMotorTorqueMsgPayload> rwMotorTorqueOutMsg;   //!< RW motor torque output message
    ReadFunctor<CmdTorqueBodyMsgPayload> vehControlInMsg;   //!<  vehicle control (Lr) Input message
    ReadFunctor<CmdTorqueBodyMsgPayload> vehControlIn2Msg;  //!<  optional vehicle control input message
    ReadFunctor<RWArrayConfigMsgPayload> rwParamsInMsg;     //!<  RW Array input message
    ReadFunctor<RWAvailabilityMsgPayload> rwAvailInMsg;     //!< optional RWs availability input message

   private:
    uint32_t numControlAxes{};      //!< [-] counter indicating how many orthogonal axes are controlled
    int numAvailRW{};               //!< [-] number of reaction wheels available
    RWArrayConfigMsgPayload
        rwConfigParams{};               //!< [-] struct to store message containing RW config parameters in body B frame
    Eigen::Matrix<double, 3, RW_EFF_CNT> G_s_B{};  //!< [-] The RW spin axis matrix in body frame components
};

#endif
