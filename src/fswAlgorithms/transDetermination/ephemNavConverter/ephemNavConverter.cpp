/*
 ISC License

 Copyright (c) 2024, Laboratory for Atmospheric Space Physics, University of Colorado at Boulder

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

#include "ephemNavConverter.h"

/*! Reset method for the module adapter interface.
 @return void
 @param callTime The clock time at which the function was called (nanoseconds)
 */
void EphemNavConverter::reset(uint64_t callTime) {
    // check if the required message has not been connected
    if (!this->ephInMsg.isLinked()) {
        this->bskLogger.bskLog(BSK_ERROR, "Error: ephemNavConverter.ephInMsg wasn't connected.");
    }
}

/*! Update method for the module adapter interface. This method also calls the algorithm update method.
 @return void
 @param callTime [ns] Time the method is called
 */
void EphemNavConverter::updateState(uint64_t callTime) {
    auto ephemMsgPayload = EphemerisMsgPayload();
    if (this->ephInMsg.isWritten()) {
        ephemMsgPayload = this->ephInMsg();
    }

    // Call the algorithm update method
    NavTransMsgPayload navTransMsgPayload = this->algorithm.update(callTime, ephemMsgPayload);

    this->stateOutMsg.write(&navTransMsgPayload, this->moduleID, callTime);
}
