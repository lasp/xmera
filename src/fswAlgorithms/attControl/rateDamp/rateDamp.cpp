// SPDX-License-Identifier: ISC
// Copyright (c) 2024, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

#include "rateDamp.h"
#include <cassert>

/*! Reset method for the BSK module adapter interface.
 @return void
 @param currentSimNanos [ns] Time the method is called
 */
void RateDamp::reset(uint64_t currentSimNanos) { assert(this->attNavInMsg.isLinked()); }

/*! Update method for the BSK module adapter interface. This method also calls the algorithm update method.
 @return void
 @param currentSimNanos [ns] Time the method is called
 */
void RateDamp::updateState(uint64_t currentSimNanos) {
    auto attNavInBuffer = NavAttMsgPayload();
    if (this->attNavInMsg.isWritten()) {
        attNavInBuffer = this->attNavInMsg();
    }

    // Call the algorithm update method
    CmdTorqueBodyMsgPayload cmdTorqueOutBuffer = this->algorithm.update(currentSimNanos, attNavInBuffer);

    this->cmdTorqueOutMsg.write(&cmdTorqueOutBuffer, this->moduleID, currentSimNanos);
}

/*! Set the module rate feedback gain
    @param double P
    @return void
    */
void RateDamp::setRateGain(double p) { this->algorithm.setRateGain(p); }

/*! Get the module rate feedback gain
    @param double measurementNoiseScale
    @return void
    */
double RateDamp::getRateGain() const { return this->algorithm.getRateGain(); }
