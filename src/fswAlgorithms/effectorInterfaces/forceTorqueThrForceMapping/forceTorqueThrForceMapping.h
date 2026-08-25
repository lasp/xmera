// SPDX-License-Identifier: ISC
// Copyright (c) 2021, Autonomous Vehicle System Lab, University of Colorado at Boulder
// Copyright (c) 2025, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

#ifndef FORCE_TORQUE_THR_FORCE_MAPPING_H
#define FORCE_TORQUE_THR_FORCE_MAPPING_H

#include "forceTorqueThrForceMappingAlgorithm.h"
#include <architecture/_GeneralModuleFiles/sys_model.h>
#include <architecture/messaging/messaging.h>
#include <architecture/msgPayloadDef/CmdForceBodyMsgPayload.h>
#include <architecture/msgPayloadDef/CmdTorqueBodyMsgPayload.h>
#include <architecture/msgPayloadDef/THRArrayCmdForceMsgPayload.h>
#include <architecture/msgPayloadDef/THRArrayConfigMsgPayload.h>
#include <architecture/msgPayloadDef/VehicleConfigMsgPayload.h>

#include <cstdint>

/*! @brief This module maps thruster forces for arbitrary forces and torques
 */
class ForceTorqueThrForceMapping final : public SysModel {
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
    ForceTorqueThrForceMappingAlgorithm algorithm{};
};

#endif
