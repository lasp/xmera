#ifndef FORCETORQUETHRFORCEMAPPING_H
#define FORCETORQUETHRFORCEMAPPING_H

#include <architecture/_GeneralModuleFiles/sys_model.h>
#include <architecture/messaging/messaging.h>
#include <architecture/msgPayloadDef/CmdForceBodyMsgPayload.h>
#include <architecture/msgPayloadDef/CmdTorqueBodyMsgPayload.h>
#include <architecture/msgPayloadDef/THRArrayCmdForceMsgPayload.h>
#include <architecture/msgPayloadDef/THRArrayConfigMsgPayload.h>
#include <architecture/msgPayloadDef/VehicleConfigMsgPayload.h>
#include <stdint.h>
#include <Eigen/Core>

/*! @brief This module maps thruster forces for arbitrary forces and torques
 */
class ForceTorqueThrForceMapping : public SysModel {
   public:
    void reset(uint64_t callTime) override;
    void updateState(uint64_t callTime) override;

    /* declare module IO interfaces */
    ReadFunctor<CmdTorqueBodyMsgPayload> cmdTorqueInMsg;    //!< (optional) vehicle control (Lr) input message
    ReadFunctor<CmdForceBodyMsgPayload> cmdForceInMsg;      //!< (optional) vehicle control force input message
    ReadFunctor<THRArrayConfigMsgPayload> thrConfigInMsg;   //!< thruster cluster configuration input message
    ReadFunctor<VehicleConfigMsgPayload> vehConfigInMsg;    //!< vehicle config input message
    Message<THRArrayCmdForceMsgPayload> thrForceCmdOutMsg;  //!< thruster force command output message

   private:
    uint32_t numThrusters{};              //!< []      The number of thrusters available on vehicle
    Eigen::Vector3d CoM_B;                      //!< [m]     CoM of the s/c
    Eigen::Matrix<double, 3, MAX_EFF_CNT> rThruster_B{Eigen::Matrix<double, 3, MAX_EFF_CNT>::Zero()};   //!< [m]     local copy of the thruster locations
    Eigen::Matrix<double, 3, MAX_EFF_CNT> gtThruster_B{Eigen::Matrix<double, 3, MAX_EFF_CNT>::Zero()};  //!< []      local copy of the thruster force unit direction vectors
};

#endif
