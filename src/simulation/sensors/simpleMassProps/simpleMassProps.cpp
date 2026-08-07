// SPDX-License-Identifier: ISC
// Copyright (c) 2021, Autonomous Vehicle System Lab, University of Colorado at Boulder
// Copyright (c) 2025, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

#include "simpleMassProps.h"

#include <architecture/utilities/linearAlgebra.h>

#include <iostream>

/*! This is the constructor for the module class. */
SimpleMassProps::SimpleMassProps() {
    return;
}

/*! Module Destructor.  */
SimpleMassProps::~SimpleMassProps() {
    return;
}

/*! This method is used to reset the module.
    @return void
 */
void SimpleMassProps::reset(uint64_t currentSimNanos) {
    // check if input message is linked
    if (!this->scMassPropsInMsg.isLinked()) {
        bskLogger.bskLog(BSK_ERROR, "simpleMassProps.scMassPropsInMsg is not linked.");
    }

    // zero the incoming message buffer
    this->scMassPropsMsgBuffer = SCMassPropsMsgPayload{};

    // call the updateState function. This make sure the vehicleConfig message is populated with the correct values upon
    // the initialization of the simulation. Some FSW modules like mrpFeedback require this, as they only set the
    // necessary mas properties on Reset and not throughout the sim.
    updateState(currentSimNanos);
}

/*! This method reads the spacecraft mass properties state input message
 */
void SimpleMassProps::readInputMessages() {
    // read the incoming power message and transfer it to the data buffer
    this->scMassPropsMsgBuffer = this->scMassPropsInMsg();

    return;
}

/*! This method writes the vehicle configuration output message.
 @return void
 @param CurrentClock The clock time associated with the model call
 */
void SimpleMassProps::writeOutputMessages(uint64_t CurrentClock) {
    // write the output message
    this->vehicleConfigOutMsg.write(this->vehicleConfigMsgBuffer, this->moduleID, CurrentClock);

    return;
}

/*! This method transfers the spacecraft mass propertiies information to a FSW format
 */
void SimpleMassProps::computeMassProperties() {
    // copy the mass value
    this->vehicleConfigMsgBuffer.massSC = this->scMassPropsMsgBuffer.massSC;

    // copy the inertia value
    for (uint64_t i = 0; i < 3; i++) {
        for (uint64_t j = 0; j < 3; j++) {
            this->vehicleConfigMsgBuffer.ISCPntB_B[3 * i + j] = this->scMassPropsMsgBuffer.ISC_PntB_B[i][j];
        }
    }

    // transfer the center of mass
    v3Copy(this->scMassPropsMsgBuffer.c_B, this->vehicleConfigMsgBuffer.CoM_B);

    return;
}

/*! This is the main method that gets called every time the module is updated. It reads the simulation message,
   transfers its contents and writes to an output FSW message.
    @return void
 */
void SimpleMassProps::updateState(uint64_t currentSimNanos) {
    readInputMessages();
    computeMassProperties();
    writeOutputMessages(currentSimNanos);

    return;
}
