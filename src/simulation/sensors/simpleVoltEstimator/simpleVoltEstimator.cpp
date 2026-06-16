// SPDX-License-Identifier: ISC
// Copyright (c) 2022, Autonomous Vehicle System Lab, University of Colorado at Boulder
// Copyright (c) 2025, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

#include "simpleVoltEstimator.h"

#include <architecture/utilities/eigenSupport.h>

#include <cstring>
#include <iostream>

/*! This is the constructor for the simple voltage estimator module.  It sets default variable
    values and initializes the various parts of the model */
SimpleVoltEstimator::SimpleVoltEstimator() {
    this->estVoltState = VoltMsgPayload{};
    this->trueVoltState = VoltMsgPayload{};
    this->PMatrix.setZero();
    this->walkBounds.setZero();
    this->errorModel = GaussMarkov<1>(this->RNGSeed);
}

/*! Destructor.  Nothing here. */
SimpleVoltEstimator::~SimpleVoltEstimator() {
    return;
}

/*! This method is used to reset the module. It
 initializes the various containers used in the model as well as creates the
 output message.  The error states are allocated as follows:
 Total states: 1
     - Voltage error [0]
 @return void
 */
void SimpleVoltEstimator::reset(uint64_t currentSimNanos) {
    // check if input message has not been included
    if (!this->voltInMsg.isLinked()) { bskLogger.bskLog(BSK_ERROR, "SimpleVoltEstimator.voltInMsg was not linked."); }

    //! - Initialize the propagation matrix to default values for use in update
    this->AMatrix.setIdentity();

    this->errorModel.setNoiseMatrix(this->PMatrix);
    this->errorModel.setRNGSeed(this->RNGSeed);
    this->errorModel.setUpperBounds(this->walkBounds);
}

/*! This method reads the input message associated with the spacecraft voltage
 */
void SimpleVoltEstimator::readInputMessages() {
    this->trueVoltState = this->voltInMsg();
}

/*! This method writes the voltage information into the output state message.
 @return void
 @param Clock The clock time associated with the model call
 */
void SimpleVoltEstimator::writeOutputMessages(uint64_t Clock) {
    this->voltOutMsg.write(&this->estVoltState, this->moduleID, Clock);
}

void SimpleVoltEstimator::applyErrors() {
    //! - Add errors
    this->estVoltState.voltage = this->trueVoltState.voltage + this->voltErrors.data()[0];
}

/*! This method sets the propagation matrix and requests new random errors from
 its GaussMarkov model.
 @return void
 */
void SimpleVoltEstimator::computeErrors() {
    VoltErrorMatrix localProp = this->AMatrix;

    //! - Set the GaussMarkov propagation matrix and compute errors
    this->errorModel.setPropMatrix(localProp);
    this->errorModel.computeNextState();
    this->voltErrors = this->errorModel.getCurrentState();
}

/*! This method calls all of the run-time operations for the simpleVoltEstimator module.
    @return void
    @param currentSimNanos The clock time associated with the model call
*/
void SimpleVoltEstimator::updateState(uint64_t currentSimNanos) {
    this->readInputMessages();
    this->computeErrors();
    this->applyErrors();
    this->writeOutputMessages(currentSimNanos);
}
