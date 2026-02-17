// SPDX-License-Identifier: ISC
// Copyright (c) 2016, Autonomous Vehicle System Lab, University of Colorado at Boulder
// Copyright (c) 2025, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

#ifndef _THR_MOMENTUM_DUMPING_H_
#define _THR_MOMENTUM_DUMPING_H_

#include <stdint.h>

#include <architecture/_GeneralModuleFiles/sys_model.h>
#include <architecture/messaging/messaging.h>
#include <architecture/msgPayloadDef/CmdTorqueBodyMsgPayload.h>
#include <architecture/msgPayloadDef/THRArrayCmdForceMsgPayload.h>
#include <architecture/msgPayloadDef/THRArrayConfigMsgPayload.h>
#include <architecture/msgPayloadDef/THRArrayOnTimeCmdMsgPayload.h>
#include <architecture/utilities/bskLogging.h>

/*! @brief thruster force momentum dumping module configuration message
 */
class ThrMomentumDumping : public SysModel {
   public:
    void reset(uint64_t callTime) override;
    void updateState(uint64_t callTime) override;
    /* declare module private variables */
    int32_t
        thrDumpingCounter;  //!<        counter to specify after how many contro period a thruster firing should occur.
    double Delta_p[MAX_EFF_CNT];             //!<        vector of desired total thruster impulses
    uint64_t lastDeltaHInMsgTime;            //!<        time tag of the last momentum change input message
    double thrOnTimeRemaining[MAX_EFF_CNT];  //!<        vector of remaining thruster on times
    uint64_t priorTime;                      //!< [ns]   Last time the attitude control is called
    int numThrusters;                        //!<        number of thrusters installed
    double thrMaxForce[MAX_EFF_CNT];         //!< [N]    vector of maximum thruster forces

    /* declare module public variables */
    int maxCounterValue;  //!<        this variable must be set to a non-zero value, indicating how many control periods
                          //!<        to wait until the thrusters fire again to dump RW momentum
    double thrMinFireTime;  //!< [s]    smallest thruster firing time
    int maxNumOfDtFiringTimes = 1;

    /* declare module IO interfaces */
    Message<THRArrayOnTimeCmdMsgPayload> thrusterOnTimeOutMsg;     //!< thruster on time output message name
    ReadFunctor<THRArrayCmdForceMsgPayload> thrusterImpulseInMsg;  //!< desired thruster impulse input message name
    ReadFunctor<THRArrayConfigMsgPayload> thrusterConfInMsg;  //!< The name of the thruster configuration Input message
    ReadFunctor<CmdTorqueBodyMsgPayload> deltaHInMsg;  //!< The name of the requested momentum change input message

    BSKLogger bskLogger = {};  //!< BSK Logging
};

#endif
