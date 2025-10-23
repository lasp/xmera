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

#include "sunlineEphem.h"

SunlineEphem::SunlineEphem() { this->algorithm = SunlineEphemAlgorithm(); }

void SunlineEphem::reset(uint64_t callTime) {
    assert(this->sunPositionInMsg.isLinked());
    assert(this->scPositionInMsg.isLinked());
    assert(this->scAttitudeInMsg.isLinked());
}

void SunlineEphem::updateState(uint64_t callTime) {
    EphemerisMsgPayload sunPos = this->sunPositionInMsg();
    NavTransMsgPayload scPos = this->scPositionInMsg();
    NavAttMsgPayload scAtt = this->scAttitudeInMsg();
    auto outputSunline = this->algorithm.updateState(sunPos, scPos, scAtt);
    this->navStateOutMsg.write(&outputSunline, this->moduleID, callTime);
}
