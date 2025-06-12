/*
 ISC License

 Copyright (c) 2016, Autonomous Vehicle Systems Lab, University of Colorado at Boulder

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
/*
    MRP_STEERING Module

 */

#include "fswAlgorithms/attControl/mrpSteering/mrpSteering.h"



/*! This method performs a complete reset of the module.  Local module variables that retain
 time varying states between function calls are reset to their default values.
 @return void
 @param callTime The clock time at which the function was called (nanoseconds)
*/
void MrpSteering::reset(uint64_t callTime)
{
    // check for required input message
    if (!this->guidInMsg.isLinked()) {
        this->bskLogger.bskLog(BSK_ERROR, "Error: mrpSteering.guidInMsg wasn't connected.");
    }

    return;
}

/*! This method takes the attitude and rate errors relative to the Reference frame, as well as
    the reference frame angular rates and acceleration
 @return void
 @param callTime The clock time at which the function was called (nanoseconds)
 */
void MrpSteering::updateState(uint64_t callTime)
{
    AttGuidMsgPayload guidCmd;      /* Guidance Message */
    RateCmdMsgPayload outMsg = {};  /* copy of output message */

    /*! - Read the dynamic input messages */
    guidCmd = this->guidInMsg();

    /*! - Call steering algorithm */
    outMsg = this->algorithm.update(callTime, guidCmd);

    /*! - Store the output message and pass it to the message bus */
    this->rateCmdOutMsg.write(&outMsg, moduleID, callTime);

    return;
}

double MrpSteering::getK1() const { return this->algorithm.getK1(); }
double MrpSteering::getK3() const { return this->algorithm.getK3(); }
double MrpSteering::getOmegaMax() const { return this->algorithm.getOmegaMax(); }
bool MrpSteering::getIgnoreOuterLoopFeedforward() const {
    return this->algorithm.getIgnoreOuterLoopFeedforward();
}
void MrpSteering::setK1(double K1) { this->algorithm.setK1(K1); }
void MrpSteering::setK3(double K3) { this->algorithm.setK3(K3); }
void MrpSteering::setOmegaMax(double omegaMax) { this->algorithm.setOmegaMax(omegaMax); }
void MrpSteering::setIgnoreOuterLoopFeedforward(bool ignore) {
    this->algorithm.setIgnoreOuterLoopFeedforward(ignore);
}


