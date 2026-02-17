// SPDX-License-Identifier: ISC
// Copyright (c) 2016, Autonomous Vehicle System Lab, University of Colorado at Boulder
// Copyright (c) 2024, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

#include "simpleBattery.h"

/*! The constructor creates a SimpleBattery instance with zero stored charge */
SimpleBattery::SimpleBattery() {
    this->storageCapacity = -1.0;
    this->storedCharge = 0.0;
    return;
}

SimpleBattery::~SimpleBattery() { return; }

/*! custom reset function.
 */
void SimpleBattery::customreset(uint64_t CurrentClock) {
    if (this->storageCapacity <= 0.0) {
        bskLogger.bskLog(BSK_ERROR, "The storageCapacity variable must be set to a positive value.");
    }
    return;
}

/*! This method integrates the current net power, and checks to see whether the integrated power falls between 0 and the
 battery's storageCapacity.
 @param *msg:  pointer to a PowerStorageStatusMsgPayload instance
 @return void
 */
void SimpleBattery::evaluateBatteryModel(PowerStorageStatusMsgPayload* msg) {
    this->storedCharge = this->storedCharge + this->currentPowerSum * (this->currentTimestep);

    if (this->storedCharge > this->storageCapacity) {
        this->storedCharge = this->storageCapacity;
    }

    if (this->storedCharge < 0) {
        this->storedCharge = 0;
    }

    msg->storageCapacity = this->storageCapacity;
    msg->storageLevel = this->storedCharge;
    msg->currentNetPower = this->currentPowerSum;

    return;
}
