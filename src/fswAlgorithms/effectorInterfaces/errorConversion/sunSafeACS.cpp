// SPDX-License-Identifier: ISC
// Copyright (c) 2016, Autonomous Vehicle System Lab, University of Colorado at Boulder
// Copyright (c) 2025, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

#include "sunSafeACS.h"
#include <architecture/utilities/rigidBodyKinematics.h>
#include <string.h>

/*! This method resets the module.
 @return void
 @param callTime The clock time at which the function was called (nanoseconds)
 */
void SunSafeACS::reset(uint64_t callTime) {
    // check if the required input messages are included
    if (!this->cmdTorqueBodyInMsg.isLinked()) {
        this->bskLogger.bskLog(BSK_ERROR, "Error: sunSafeACS.cmdTorqueBodyInMsg wasn't connected.");
    }
}

/*! This method takes the estimated body-observed sun vector and computes the
 current attitude/attitude rate errors to pass on to control.
 @return void
 @param callTime The clock time at which the function was called (nanoseconds)
 */
void SunSafeACS::updateState(uint64_t callTime) {
    CmdTorqueBodyMsgPayload cntrRequest;

    /*! - Read the input parsed CSS sensor data message*/
    cntrRequest = this->cmdTorqueBodyInMsg();
    computeSingleThrustBlock(&(this->thrData), callTime, &cntrRequest, moduleID);

    return;
}
