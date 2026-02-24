// SPDX-License-Identifier: ISC
// Copyright (c) 2016, Autonomous Vehicle System Lab, University of Colorado at Boulder

#include "simplePowerSink.h"

/*! Constructor, which sets the default nodePowerOut to zero.
 */
SimplePowerSink::SimplePowerSink() {
    this->nodePowerOut = 0.0;
    return;
}

SimplePowerSink::~SimplePowerSink() { return; }

/*! Loads the nodePowerOut attribute into the powerUsageSimMessage instance.
 */
void SimplePowerSink::evaluatePowerModel(PowerNodeUsageMsgPayload* powerUsageSimMsg) {
    powerUsageSimMsg->netPower = this->nodePowerOut;

    return;
}
