/*
 ISC License

 Copyright (c) 2024, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

 Permission to use, copy, modify, and/or distribute this software for any
 purpose with or without fee is hereby granted, provided that the above
 copyright notice and this permission notice appear in all copies.

 THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
 WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
 MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
 ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
 WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
 ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
 OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.

 */

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
void RateDamp::setRateGain(const double p) { this->algorithm.setRateGain(p); }

/*! Get the module rate feedback gain
    @param double measurementNoiseScale
    @return void
    */
double RateDamp::getRateGain() const { return this->algorithm.getRateGain(); }
