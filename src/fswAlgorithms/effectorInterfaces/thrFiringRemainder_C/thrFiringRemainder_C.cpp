// SPDX-License-Identifier: ISC
// Copyright (c) 2016, Autonomous Vehicle System Lab, University of Colorado at Boulder

#include "thrFiringRemainder_C.h"
#include "thrFiringRemainderAlgorithm_C.h"

void ThrFiringRemainder_C::reset(const uint64_t callTime) {
    // check if the required input messages are included
    if (!this->thrConfInMsg.isLinked()) {
        this->bskLogger.bskLog(BSK_ERROR, "Error: thrFiringRemainder.thrConfInMsg wasn't connected.");
    }
    if (!this->thrForceInMsg.isLinked()) {
        this->bskLogger.bskLog(BSK_ERROR, "Error: thrFiringRemainder.thrForceInMsg wasn't connected.");
    }

    /*! - read in the support messages */
    const THRArrayConfigMsgPayload localThrusterData = this->thrConfInMsg();
    ::reset(&this->algorithmState, localThrusterData);
}

void ThrFiringRemainder_C::updateState(uint64_t callTime) {
    const THRArrayCmdForceMsgPayload thrForceIn = this->thrForceInMsg();

    THRArrayOnTimeCmdMsgPayload thrOnTimeOut = ::updateState(&this->algorithmState, callTime, thrForceIn);

    this->onTimeOutMsg.write(&thrOnTimeOut, this->moduleID, callTime);
}

/*! Setter method for thrMinFireTime.
 @return void
 @param thrMinFireTime
*/
void ThrFiringRemainder_C::setThrMinFireTime(const double thrMinFireTime) {
    this->algorithmState.thrMinFireTime = thrMinFireTime;
}

/*! Getter method for thrMinFireTime.
 @return const double
*/
double ThrFiringRemainder_C::getThrMinFireTime() const { return this->algorithmState.thrMinFireTime; }

/*! Setter method for baseThrustState.
 @return void
 @param baseThrustState
*/
void ThrFiringRemainder_C::setBaseThrustState(const int baseThrustState) {
    this->algorithmState.baseThrustState = baseThrustState;
}

/*! Getter method for baseThrustState.
 @return const int
*/
int ThrFiringRemainder_C::getBaseThrustState() const { return this->algorithmState.baseThrustState; }

/*! Setter method for defaultControlPeriod.
 @return void
 @param defaultControlPeriod
*/
void ThrFiringRemainder_C::setDefaultControlPeriod(const double defaultControlPeriod) {
    this->algorithmState.defaultControlPeriod = defaultControlPeriod;
}

/*! Getter method for defaultControlPeriod.
 @return const double
*/
double ThrFiringRemainder_C::getDefaultControlPeriod() const { return this->algorithmState.defaultControlPeriod; }
