// SPDX-License-Identifier: ISC
// Copyright (c) 2016, Autonomous Vehicle System Lab, University of Colorado at Boulder
// Copyright (c) 2024, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

#include "ephemNavConverter.h"

/*! Reset method for the module adapter interface.
 @return void
 @param callTime The clock time at which the function was called (nanoseconds)
 */
void EphemNavConverter::reset(uint64_t callTime) {
    // check if the required message has not been connected
    if (!this->ephInMsg.isLinked()) {
        this->bskLogger.bskLog(BSK_ERROR, "Error: ephemNavConverter.ephInMsg wasn't connected.");
    }
}

/*! Update method for the module adapter interface. This method also calls the algorithm update method.
 @return void
 @param callTime [ns] Time the method is called
 */
void EphemNavConverter::updateState(uint64_t callTime) {
    auto ephemMsgPayload = EphemerisMsgPayload();
    if (this->ephInMsg.isWritten()) {
        ephemMsgPayload = this->ephInMsg();
    }

    // Call the algorithm update method
    NavTransMsgPayload navTransMsgPayload = this->algorithm.update(callTime, ephemMsgPayload);

    this->stateOutMsg.write(&navTransMsgPayload, this->moduleID, callTime);
}
