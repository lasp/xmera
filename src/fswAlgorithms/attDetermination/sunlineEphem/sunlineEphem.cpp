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
