/*
 ISC License

 Copyright (c) 2025, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

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

#ifndef FORCE_TORQUE_THR_FORCE_MAPPING_C_H
#define FORCE_TORQUE_THR_FORCE_MAPPING_C_H

#include <architecture/_GeneralModuleFiles/sys_model.h>
#include <architecture/messaging/messaging.h>
#include <architecture/msgPayloadDef/CmdForceBodyMsgPayload.h>
#include <architecture/msgPayloadDef/CmdTorqueBodyMsgPayload.h>
#include <architecture/msgPayloadDef/THRArrayCmdForceMsgPayload.h>
#include <architecture/msgPayloadDef/THRArrayConfigMsgPayload.h>
#include <architecture/msgPayloadDef/VehicleConfigMsgPayload.h>
#include <architecture/utilities/bskLogging.h>
#include <stdint.h>

/*! @brief This module maps thruster forces for arbitrary forces and torques
 */
class ForceTorqueThrForceMapping_C : public SysModel {
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

    BSKLogger bskLogger = {};  //!< BSK Logging
};

#endif
