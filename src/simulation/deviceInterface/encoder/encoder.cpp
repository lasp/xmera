// SPDX-License-Identifier: ISC
// Copyright (c) 2021, Autonomous Vehicle System Lab, University of Colorado at Boulder
// Copyright (c) 2025, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

#include "encoder.h"

#include <architecture/utilities/macroDefinitions.h>
#include <architecture/utilities/simDefinitions.h>

#include <cmath>
#include <numbers>

/*! This is the constructor for the module class.  It sets default variable
    values and initializes the various parts of the model */
Encoder::Encoder() {
    this->numRW = -1;  // set the number of reaction wheels to -1 to throw a warning if not set
    this->clicksPerRotation = -1;
}

/*! This method is used to reset the module.
 @return void
 */
void Encoder::reset(uint64_t currentSimNanos) {
    // check if input message is linked
    if (!this->rwSpeedInMsg.isLinked()) { bskLogger.bskLog(BSK_ERROR, "encoder.rwSpeedInMsg is not linked."); }

    // if the number of clicks is not greater than 0, throw a warning message
    if (this->clicksPerRotation <= 0) {
        bskLogger.bskLog(BSK_ERROR, "encoder: number of clicks must be a positive integer.");
    }

    // if the number of reaction wheels is not greater than 0, throw a warning message
    if (this->numRW <= 0) {
        bskLogger.bskLog(
            BSK_ERROR,
            "encoder: number of reaction wheels must be a positive integer. It may not have been set."
        );
    }

    // reset the previous time
    this->prevTime = currentSimNanos;

    // zero the RW wheel output message buffer //
    this->rwSpeedConverted = RWSpeedMsgPayload{};

    // Loop through the RW to set some internal parameters to default
    for (int i = 0; i < RW_EFF_CNT; i++) {
        // set all reaction wheels signal to nominal
        this->rwSignalState[i] = SIGNAL_NOMINAL;
        // set the remaining clicks to zero
        this->remainingClicks[i] = 0.0;
    }
}

/*! This method reads the speed input message
 */
void Encoder::readInputMessages() {
    // read the incoming power message
    this->rwSpeedBuffer = this->rwSpeedInMsg();
}

/*! This method writes encoded the wheel speed message.
 @return void
 @param CurrentClock The clock time associated with the model call
 */
void Encoder::writeOutputMessages(uint64_t CurrentClock) {
    this->rwSpeedOutMsg.write(this->rwSpeedConverted, this->moduleID, CurrentClock);
}

/*! This method applies an encoder to the reaction wheel speeds.
 */
void Encoder::encode(uint64_t currentSimNanos) {
    // convert clicks per rotation to clicks per radian
    double const clicksPerRadian = this->clicksPerRotation / (2 * std::numbers::pi);

    // set the time step
    double const timeStep = (currentSimNanos - this->prevTime) * NANO2SEC;

    // at the beginning of the simulation, the encoder simply outputs the true RW speeds
    if (timeStep == 0.0) {
        this->rwSpeedConverted = this->rwSpeedInMsg();
    } else {
        // loop through the RW
        for (int i = 0; i < this->numRW; i++) {
            // check if encoder is operational
            if (this->rwSignalState[i] == SIGNAL_NOMINAL) {
                // calculate the angle sweeped by the reaction wheel during the time step
                double const angle = this->rwSpeedBuffer.wheelSpeeds[i] * timeStep;

                // calculate the number of clicks
                double const totalClicks = angle * clicksPerRadian + this->remainingClicks[i];
                double const numberClicks = std::trunc(totalClicks);

                // update the remaining clicks
                this->remainingClicks[i] = totalClicks - numberClicks;

                // calculate the discretized angular velocity
                this->rwSpeedConverted.wheelSpeeds[i] = numberClicks / (clicksPerRadian * timeStep);
            }
            // check if encoder is off
            else if (this->rwSignalState[i] == SIGNAL_OFF) {
                // set the outgoing reaction wheel speed to 0
                this->rwSpeedConverted.wheelSpeeds[i] = 0.0;

                // reset the remaining clicks
                this->remainingClicks[i] = 0;
            } else if (this->rwSignalState[i] == SIGNAL_STUCK) {
                // if the encoder is stuck, it will output the previous results
            } else {
                bskLogger
                    .bskLog(BSK_ERROR, "encoder: un-modeled encoder signal mode %d selected.", this->rwSignalState[i]);
            }
        }
    }
}

/*! This method runs the encoder module in the sim.
 */
void Encoder::updateState(uint64_t currentSimNanos) {
    this->readInputMessages();
    this->encode(currentSimNanos);
    this->writeOutputMessages(currentSimNanos);

    this->prevTime = currentSimNanos;
}
