// SPDX-License-Identifier: ISC
// Copyright (c) 2016, Autonomous Vehicle System Lab, University of Colorado at Boulder
// Copyright (c) 2025, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

#include "powerNodeBase.h"

#include <architecture/utilities/astroConstants.h>
#include <architecture/utilities/macroDefinitions.h>

/*! This method initializes the messaging parameters to either empty strings for message names or -1 for message IDs.
 @return void
 */
PowerNodeBase::PowerNodeBase() {
    this->powerStatus = 1;                            //! Node defaults to on unless overwritten.
    this->nodePowerMsg = PowerNodeUsageMsgPayload{};  //! Power node message is zero by default.
    return;
}

/*! Destructor.
 @return void
 */
PowerNodeBase::~PowerNodeBase() {
    return;
}

/*! This method is used to reset the module. In general, no functionality must be reset.
 @return void
 */
void PowerNodeBase::reset(uint64_t currentSimNanos) {
    //! - call the custom environment module reset method
    customreset(currentSimNanos);

    return;
}

/*! This method writes out a message.
 @return void
 */
void PowerNodeBase::writeMessages(uint64_t CurrentClock) {
    //! - write power output message
    this->nodePowerOutMsg.write(this->nodePowerMsg, this->moduleID, CurrentClock);

    //! - call the custom method to perform additional output message writing
    customWriteMessages(CurrentClock);

    return;
}

/*! This method is used to read incoming power status messages.
 @return void
 */
bool PowerNodeBase::readMessages() {
    DeviceStatusMsgPayload statusMsg;

    //! - read in the power node use/supply messages
    bool powerRead = true;
    bool tmpStatusRead = true;
    if (this->nodeStatusInMsg.isLinked()) {
        statusMsg = this->nodeStatusInMsg();
        tmpStatusRead = this->nodeStatusInMsg.isWritten();
        this->nodeStatusMsg = statusMsg;
        this->powerStatus = this->nodeStatusMsg.deviceStatus;
        powerRead = powerRead && tmpStatusRead;
    }

    //! - call the custom method to perform additional input reading
    bool customRead = this->customReadMessages();

    return (powerRead && customRead);
}

/*! Core compute operation that implements switching logic and computes module-wise power consumption.
 */

void PowerNodeBase::computePowerStatus(double currentTime) {
    if (this->powerStatus > 0) {
        this->evaluatePowerModel(&this->nodePowerMsg);
    } else {
        this->nodePowerMsg = PowerNodeUsageMsgPayload{};
    }

    return;
}

/*! Provides logic for running the read / compute / write operation that is the module's function.
 @param currentSimNanos The current simulation time in nanoseconds
 */
void PowerNodeBase::updateState(uint64_t currentSimNanos) {
    //! - Only update the power status if we were able to read in messages.
    if (this->readMessages()) {
        this->computePowerStatus(currentSimNanos * NANO2SEC);
    } else {
        /* if the read was not successful then zero the output message */
        this->nodePowerMsg = PowerNodeUsageMsgPayload{};
    }

    this->writeMessages(currentSimNanos);

    return;
}

/*! Custom reset() method.  This allows a child class to add additional functionality to the reset() method
 @return void
 */
void PowerNodeBase::customreset(uint64_t CurrentClock) {
    return;
}

/*! custom Write method, similar to customSelfInit.
 @return void
 */
void PowerNodeBase::customWriteMessages(uint64_t CurrentClock) {
    return;
}

/*! Custom read method, similar to customSelfInit; returns `true' by default.
 @return void
 */
bool PowerNodeBase::customReadMessages() {
    return true;
}
