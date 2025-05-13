/*
 ISC License

 Copyright (c) 2025, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

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

#include "fswAlgorithms/attControl/rateDamp/rateDampAlgorithm.h"

/*! This method is used to reset the module.
 @return void
 */
void RateDampAlgorithm::reset(uint64_t currentSimNanos) {
    // Reset the algorithm
}

/*! This method is the main carrier for the computation of the control torque.
 @return void
 @param currentSimNanos The current simulation time for system
 */
CmdTorqueBodyMsgPayload RateDampAlgorithm::update(uint64_t currentSimNanos, NavAttMsgPayload& attNavInMsg) {
    /*! Create and populate cmd torque buffer message */
    CmdTorqueBodyMsgPayload cmdTorqueOutBuffer{};
    for (int i = 0; i < 3; ++i) {
        cmdTorqueOutBuffer.torqueRequestBody[i] = -this->P * attNavInMsg.omega_BN_B[i];
    }

    return cmdTorqueOutBuffer;
}

/*! Set the module rate feedback gain
    @param double P
    @return void
    */
void RateDampAlgorithm::setRateGain(const double p) { this->P = p; }

/*! Get the module rate feedback gain
    @return double
    */
double RateDampAlgorithm::getRateGain() const { return this->P; }
