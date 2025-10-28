#ifndef FORCE_TORQUE_THR_FORCE_MAPPING_H
#define FORCE_TORQUE_THR_FORCE_MAPPING_H

#include <cstdint>
#include <architecture/_GeneralModuleFiles/sys_model.h>
#include <architecture/messaging/messaging.h>
#include <architecture/msgPayloadDef/CmdForceBodyMsgPayload.h>
#include <architecture/msgPayloadDef/CmdTorqueBodyMsgPayload.h>
#include <architecture/msgPayloadDef/THRArrayCmdForceMsgPayload.h>
#include <architecture/msgPayloadDef/THRArrayConfigMsgPayload.h>
#include <architecture/msgPayloadDef/VehicleConfigMsgPayload.h>
#include "forceTorqueThrForceMappingAlgorithm.h"

/*! @brief This module maps thruster forces for arbitrary forces and torques
 */
class ForceTorqueThrForceMapping : public SysModel {
   public:
    ForceTorqueThrForceMapping() = default;
    ~ForceTorqueThrForceMapping() final = default;

    void reset(uint64_t callTime) override;
    void updateState(uint64_t callTime) override;

    /* declare module IO interfaces */
    ReadFunctor<CmdTorqueBodyMsgPayload> cmdTorqueInMsg;    //!< (optional) vehicle control (Lr) input message
    ReadFunctor<CmdForceBodyMsgPayload> cmdForceInMsg;      //!< (optional) vehicle control force input message
    ReadFunctor<THRArrayConfigMsgPayload> thrConfigInMsg;   //!< thruster cluster configuration input message
    ReadFunctor<VehicleConfigMsgPayload> vehConfigInMsg;    //!< vehicle config input message
    Message<THRArrayCmdForceMsgPayload> thrForceCmdOutMsg;  //!< thruster force command output message

   private:
    ForceTorqueThrForceMappingAlgorithm algorithm{};
};

#endif
