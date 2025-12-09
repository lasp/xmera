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

#ifndef RW_MOTOR_TORQUE_C_H
#define RW_MOTOR_TORQUE_C_H

#include <stdint.h>

#include <architecture/_GeneralModuleFiles/sys_model.h>
#include <architecture/messaging/messaging.h>
#include <architecture/msgPayloadDef/CmdTorqueBodyMsgPayload.h>
#include <architecture/msgPayloadDef/RWArrayConfigMsgPayload.h>
#include <architecture/msgPayloadDef/RWAvailabilityMsgPayload.h>
#include <architecture/msgPayloadDef/RwMotorTorqueMsgPayload.h>

#include <architecture/utilities/bskLogging.h>

/*! @brief Top level structure for the sub-module routines. */
class RwMotorTorque_C : public SysModel {
   public:
    void reset(uint64_t callTime) override;
    void updateState(uint64_t callTime) override;
    /* declare module private variables */
    double controlAxes_B[3 * 3];  //!< [-] array of the control unit axes
    uint32_t numControlAxes;      //!< [-] counter indicating how many orthogonal axes are controlled
    int numAvailRW;               //!< [-] number of reaction wheels available
    RWArrayConfigMsgPayload
        rwConfigParams;                 //!< [-] struct to store message containing RW config parameters in body B frame
    double GsMatrix_B[3 * RW_EFF_CNT];  //!< [-] The RW spin axis matrix in body frame components
    double CGs[3][RW_EFF_CNT];          //!< [-] Projection matrix that defines the controlled body axes

    /* declare module IO interfaces */
    Message<RwMotorTorqueMsgPayload> rwMotorTorqueOutMsg;   //!< RW motor torque output message
    ReadFunctor<CmdTorqueBodyMsgPayload> vehControlInMsg;   //!<  vehicle control (Lr) Input message
    ReadFunctor<CmdTorqueBodyMsgPayload> vehControlIn2Msg;  //!<  optional vehicle control input message
    ReadFunctor<RWArrayConfigMsgPayload> rwParamsInMsg;     //!<  RW Array input message
    ReadFunctor<RWAvailabilityMsgPayload> rwAvailInMsg;     //!< optional RWs availability input message

    BSKLogger bskLogger = {};  //!< BSK Logging
};

#endif
