// SPDX-License-Identifier: ISC
// Copyright (c) 2016, Autonomous Vehicle System Lab, University of Colorado at Boulder

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
