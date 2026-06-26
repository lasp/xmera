// SPDX-License-Identifier: ISC
// Copyright (c) 2016, Autonomous Vehicle System Lab, University of Colorado at Boulder
// Copyright (c) 2025, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

#include "extForceTorque.h"

#include <architecture/utilities/eigenSupport.h>

#include <iostream>

/*! This is the constructor.  It sets some default initializers that can be
 overriden by the user.*/
ExtForceTorque::ExtForceTorque() {
    /* initialize the 3 output vectors to zero */
    this->extForce_N.fill(0.0);
    this->extForce_B.fill(0.0);
    this->extTorquePntB_B.fill(0.0);

    return;
}

/*! The destructor.  Nothing of note is performed here*/
ExtForceTorque::~ExtForceTorque() {
    return;
}

/*! This method is used to reset the module.
 @return void
 */
void ExtForceTorque::reset(uint64_t currentSimNanos) {
    /* zero the input messages */
    this->incomingCmdTorqueBuffer = CmdTorqueBodyMsgPayload{};
    this->incomingCmdForceBodyBuffer = CmdForceBodyMsgPayload{};
    this->incomingCmdForceInertialBuffer = CmdForceInertialMsgPayload{};
}

void ExtForceTorque::linkInStates(DynParamManager &statesIn) {}

/*! This module does not write any output messages.
 @param currentClock The current time used for time-stamping the message
 @return void
 */
void ExtForceTorque::writeOutputMessages(uint64_t currentClock) {}

/*! This method is used to read the incoming message and set the
 associated buffer structure.
 @return void
 */
void ExtForceTorque::readInputMessages() {
    if (this->cmdTorqueInMsg.isLinked()) { this->incomingCmdTorqueBuffer = this->cmdTorqueInMsg(); }
    if (this->cmdForceBodyInMsg.isLinked()) { this->incomingCmdForceBodyBuffer = this->cmdForceBodyInMsg(); }
    if (this->cmdForceInertialInMsg.isLinked()) {
        this->incomingCmdForceInertialBuffer = this->cmdForceInertialInMsg();
    }
}

/*! This method is used to compute the RHS forces and torques.
    Note:   the module can set any of these three vecors, or a subset.  Regarding the external force, the
            matrix representations in the body (B) and inerial (N) frame components are treated as 2
            separate vectors.  Only set both if you mean to, as both vectors will be included.
 */
void ExtForceTorque::computeForceTorque(double integTime, double timeStep) {
    Eigen::Vector3d cmdVec;

    /* add the cmd force in inertial frame components set via Python */
    this->forceExternal_N = this->extForce_N;
    /* add the cmd force in inertial frame components set via FSW communication */
    if (this->cmdForceInertialInMsg.isLinked()) {
        cmdVec = cArrayToEigenVector(this->incomingCmdForceInertialBuffer.forceRequestInertial);
        this->forceExternal_N += cmdVec;
    }

    /* add the cmd force in body frame components set via Python */
    this->forceExternal_B = this->extForce_B;
    /* add the cmd force in body frame components set via FSW communication */
    if (this->cmdForceBodyInMsg.isLinked()) {
        cmdVec = cArrayToEigenVector(this->incomingCmdForceBodyBuffer.forceRequestBody);
        this->forceExternal_B += cmdVec;
    }

    /* add the cmd torque about Point B in body frame components set via Python */
    this->torqueExternalPntB_B = this->extTorquePntB_B;
    /* add the cmd torque about Point B in body frame components set via FSW communication */
    if (this->cmdTorqueInMsg.isLinked()) {
        cmdVec = cArrayToEigenVector(this->incomingCmdTorqueBuffer.torqueRequestBody);
        this->torqueExternalPntB_B += cmdVec;
    }

    return;
}

void ExtForceTorque::updateState(uint64_t currentSimNanos) {
    this->readInputMessages();
}
