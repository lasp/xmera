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

/*! @brief This module maps thruster forces for arbitrary forces and torques
 */
class ForceTorqueThrForceMapping : public SysModel {
   public:
    void reset(uint64_t callTime) override;
    void updateState(uint64_t callTime) override;
    /* declare module public variables */
    double rThruster_B[MAX_EFF_CNT][3];   //!< [m]     local copy of the thruster locations
    double gtThruster_B[MAX_EFF_CNT][3];  //!< []      local copy of the thruster force unit direction vectors

    /* declare module private variables */
    uint32_t numThrusters;  //!< []      The number of thrusters available on vehicle
    double CoM_B[3];        //!< [m]     CoM of the s/c

    /* declare module IO interfaces */
    ReadFunctor<CmdTorqueBodyMsgPayload> cmdTorqueInMsg;    //!< (optional) vehicle control (Lr) input message
    ReadFunctor<CmdForceBodyMsgPayload> cmdForceInMsg;      //!< (optional) vehicle control force input message
    ReadFunctor<THRArrayConfigMsgPayload> thrConfigInMsg;   //!< thruster cluster configuration input message
    ReadFunctor<VehicleConfigMsgPayload> vehConfigInMsg;    //!< vehicle config input message
    Message<THRArrayCmdForceMsgPayload> thrForceCmdOutMsg;  //!< thruster force command output message
};

#endif
