// SPDX-License-Identifier: ISC
// Copyright (c) 2023, Autonomous Vehicle System Lab, University of Colorado at Boulder
// Copyright (c) 2025, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

#ifndef _TORQUE_SCHEDULER_
#define _TORQUE_SCHEDULER_

#include <architecture/_GeneralModuleFiles/sys_model.h>
#include <architecture/messaging/messaging.h>
#include <architecture/msgPayloadDef/ArrayEffectorLockMsgPayload.h>
#include <architecture/msgPayloadDef/ArrayMotorTorqueMsgPayload.h>
#include <architecture/utilities/bskLogging.h>
#include <stdint.h>

/*! @brief Top level structure for the sub-module routines. */
class TorqueScheduler : public SysModel {
   public:
    void reset(uint64_t callTime) override;
    void updateState(uint64_t callTime) override;

    /* declare these user-defined inputs */
    int lockFlag;    //!< flag to control the scheduler logic
    double tSwitch;  //!< [s] time span after t0 at which controller switches to second angle

    /* declare this quantity that is a module internal variable */
    uint64_t t0;  //!< [ns] epoch time where module is reset

    /* declare module IO interfaces */
    ReadFunctor<ArrayMotorTorqueMsgPayload> motorTorque1InMsg;  //!< input motor torque message #1
    ReadFunctor<ArrayMotorTorqueMsgPayload> motorTorque2InMsg;  //!< input motor torque message #1
    Message<ArrayMotorTorqueMsgPayload>
        motorTorqueOutMsg;  //!< output msg containing the motor torque to the array drive
    Message<ArrayEffectorLockMsgPayload>
        effectorLockOutMsg;  //!< output msg containing the flag to actuate or lock the motor

    BSKLogger bskLogger = {};  //!< BSK Logging
};

#endif
