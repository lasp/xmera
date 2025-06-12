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
#include "architecture/utilities/avsEigenSupport.h"


/*! This method performs a complete reset of the module.  Local module variables that retain
 time varying states between function calls are reset to their default values.
 @return void
 @param callTime The clock time at which the function was called (nanoseconds)
*/
void MrpSteering::reset(uint64_t callTime)
{
    if (!this->guidInMsg.isLinked()) {
        this->bskLogger.bskLog(BSK_ERROR, "Error: mrpSteering.guidInMsg wasn't connected.");
    }

    this->algorithm.reset(callTime);
}

/*! This method takes the attitude and rate errors relative to the Reference frame, as well as
    the reference frame angular rates and acceleration
 @return void
 @param callTime The clock time at which the function was called (nanoseconds)
 */
void MrpSteering::updateState(uint64_t callTime)
{
    AttGuidMsgPayload guidCmd = {};
    if (this->guidInMsg.isWritten()) {
        guidCmd = this->guidInMsg();
    }

    RateCmdMsgPayload outMsg = this->algorithm.update(callTime, guidCmd);

    this->rateCmdOutMsg.write(&outMsg, moduleID, callTime);
}

void MrpSteering::setK1(double k1) { this->algorithm.setK1(k1); }
double MrpSteering::getK1() const { return this->algorithm.getK1(); }
void MrpSteering::setK3(double k3) { this->algorithm.setK3(k3); }
double MrpSteering::getK3() const { return this->algorithm.getK3(); }
void MrpSteering::setOmegaMax(double omegaMax) { this->algorithm.setOmegaMax(omegaMax); }
double MrpSteering::getOmegaMax() const { return this->algorithm.getOmegaMax(); }
void MrpSteering::setIgnoreOuterLoopFeedforward(bool flag) { this->algorithm.setIgnoreOuterLoopFeedforward(flag); }
bool MrpSteering::getIgnoreOuterLoopFeedforward() const { return this->algorithm.getIgnoreOuterLoopFeedforward(); }

