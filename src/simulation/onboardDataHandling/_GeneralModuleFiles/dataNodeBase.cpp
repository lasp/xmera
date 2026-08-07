// SPDX-License-Identifier: ISC
// Copyright (c) 2016, Autonomous Vehicle System Lab, University of Colorado at Boulder
// Copyright (c) 2025, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

#include "dataNodeBase.h"

#include <architecture/utilities/macroDefinitions.h>

#include <string.h>

/*! Constructor.
 @return void
 */
DataNodeBase::DataNodeBase() {
    this->dataStatus = 1;  //!< Node defaults to on unless overwritten.
    this->nodeDataMsg = DataNodeUsageMsgPayload{};

    return;
}

/*! Destructor.
 @return void
 */
DataNodeBase::~DataNodeBase() {
    return;
}

/*! This method is used to reset the module. In general, no functionality is reset.
 @param currentSimNanos
 @return void
 */
void DataNodeBase::reset(uint64_t currentSimNanos) {
    //! - call the custom environment module reset method
    customreset(currentSimNanos);

    return;
}

/*! This method writes out the data node messages (dataName, baudRate)
 @param CurrentClock
 @return void
 */
void DataNodeBase::writeMessages(uint64_t CurrentClock) {
    //! - write dataNode output messages - baud rate and name
    this->nodeDataOutMsg.write(this->nodeDataMsg, this->moduleID, CurrentClock);

    //! - call the custom method to perform additional output message writing
    customWriteMessages(CurrentClock);
    return;
}

/*! This method reads the device status messages and calls a customReadMessages method
 @return bool
 */
bool DataNodeBase::readMessages() {
    //! - read in the data node use/supply messages
    bool dataRead = true;
    bool tmpStatusRead = true;
    if (this->nodeStatusInMsg.isLinked()) {
        this->nodeStatusMsg = this->nodeStatusInMsg();
        this->dataStatus = this->nodeStatusMsg.deviceCmd;
        tmpStatusRead = this->nodeStatusInMsg.isWritten();
        dataRead = dataRead && tmpStatusRead;
    }

    //! - call the custom method to perform additional input reading
    bool customRead = this->customReadMessages();
    return (dataRead && customRead);
}

/*! This method evaluates the implementation-specific data model if the device is set to on.
 @param CurrentTime
 @return void
 */
void DataNodeBase::computeDataStatus(double CurrentTime) {
    if (this->dataStatus > 0) {
        this->evaluateDataModel(&this->nodeDataMsg, CurrentTime);
    } else {
        this->nodeDataMsg = DataNodeUsageMsgPayload{};
    }
    return;
}

/*! This method updates the state by reading messages, calling computeDataStatus, and writing messages
 @param currentSimNanos
 @return void
 */
void DataNodeBase::updateState(uint64_t currentSimNanos) {
    //! - Only update the data status if we were able to read in messages.
    if (this->readMessages()) {
        this->computeDataStatus(currentSimNanos * NANO2SEC);
    } else {
        //! - If the read was not successful then zero the output message
        this->nodeDataMsg = DataNodeUsageMsgPayload{};
    }

    this->writeMessages(currentSimNanos);
    return;
}

/*! Custom reset() method.  This allows a child class to add additional functionality to the reset() method
 @return void
 */
void DataNodeBase::customreset(uint64_t CurrentClock) {
    return;
}

/*! custom Write method, similar to customSelfInit.
 @return void
 */
void DataNodeBase::customWriteMessages(uint64_t CurrentClock) {
    return;
}

/*! Custom read method, similar to customSelfInit; returns `true' by default.
 @return void
 */
bool DataNodeBase::customReadMessages() {
    return true;
}
