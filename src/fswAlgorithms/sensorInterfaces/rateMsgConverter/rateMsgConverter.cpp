// SPDX-License-Identifier: ISC
// Copyright (c) 2016, Autonomous Vehicle System Lab, University of Colorado at Boulder
// Copyright (c) 2025, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

/*
    Rate Converter message

    Note:   this module reads in a message of type ImuSensorBodyMsgPayload, extracts the body rate vector information,
            and adds this info to a msg of type NavAttMsgPayload.
    Author: Hanspeter Schaub
    Date:   June 30, 2018

 */

#include <string.h>
#include "rateMsgConverter.h"
#include <architecture/utilities/linearAlgebra.h>

/*! This method performs a complete reset of the module.  Local module variables that retain
 time varying states between function calls are reset to their default values.
 @return void
 @param callTime The clock time at which the function was called (nanoseconds)
 */
void RateMsgConverter::reset(uint64_t callTime) {
    // check if the required message has not been connected
    if (!this->imuRateInMsg.isLinked()) {
        this->bskLogger.bskLog(BSK_ERROR, "Error: rateMsgConverter.imuRateInMsg wasn't connected.");
    }
}

/*! This method performs a time step update of the module.
 @return void
 @param callTime The clock time at which the function was called (nanoseconds)
 */
void RateMsgConverter::updateState(uint64_t callTime) {
    /*! - read in the message of type IMUSensorBodyMsgPayload */
    IMUSensorBodyMsgPayload inMsg = this->imuRateInMsg();

    /*! - create a zero message of type NavAttMsgPayload which has the rate vector from the input message */
    NavAttMsgPayload outMsg = {};
    v3Copy(inMsg.AngVelBody, outMsg.omega_BN_B);

    /*! - write output message */
    this->navRateOutMsg.write(outMsg, this->moduleID, callTime);

    return;
}
