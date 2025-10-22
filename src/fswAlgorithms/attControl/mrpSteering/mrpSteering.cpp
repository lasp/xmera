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

#include "mrpSteering.h"
#include <stdexcept>

/*! This method performs a complete reset of the module.  Local module variables that retain
 time varying states between function calls are reset to their default values.
 @return void
 @param callTime The clock time at which the function was called (nanoseconds)
*/
void MrpSteering::reset(uint64_t callTime) {
    // check for required input message
    if (!this->guidInMsg.isLinked()) {
        throw std::invalid_argument("mrpSteering.guidInMsg wasn't connected.");
    }
}

/*! This method takes the attitude and rate errors relative to the Reference frame, as well as
    the reference frame angular rates and acceleration
 @return void
 @param callTime The clock time at which the function was called (nanoseconds)
 */
void MrpSteering::updateState(uint64_t callTime) {
    AttGuidMsgPayload guidCmd = this->guidInMsg();

    RateCmdMsgPayload outMsg = this->algorithm.update(guidCmd);

    this->rateCmdOutMsg.write(&outMsg, moduleID, callTime);
}

/*! Set the linear feedback gain K1
 @return void
 @param gain [-] linear feedback gain K1
*/
void MrpSteering::setK1(const double gain) { this->algorithm.setK1(gain); }

/*! Get the linear feedback gain K1
 @return double
*/
double MrpSteering::getK1() const { return this->algorithm.getK1(); }

/*! Set the cubic feedback gain K3
 @return void
 @param gain [-] cubic feedback gain K3
*/
void MrpSteering::setK3(const double gain) { this->algorithm.setK3(gain); }

/*! Get the cubic feedback gain K3
 @return double
*/
double MrpSteering::getK3() const { return this->algorithm.getK3(); }

/*! Set the maximum rate command of steering control
 @return void
 @param omega [-] maximum rate command of steering control
*/
void MrpSteering::setOmegaMax(const double omega) { this->algorithm.setOmegaMax(omega); }

/*! Get the maximum rate command of steering control
 @return double
*/
double MrpSteering::getOmegaMax() const { return this->algorithm.getOmegaMax(); }

/*! Set whether the outer loop feed-forward is ignored
 @return void
 @param ignore boolean whether the outer loop feed-forward should be ignored
*/
void MrpSteering::setIgnoreFeedforward(const bool ignore) { this->algorithm.setIgnoreFeedforward(ignore); }

/*! Get whether the outer loop feed-forward is ignored
 @return bool
*/
bool MrpSteering::getIgnoreFeedforward() const { return this->algorithm.getIgnoreFeedforward(); }
