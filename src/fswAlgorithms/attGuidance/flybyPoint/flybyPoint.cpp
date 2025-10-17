#include "flybyPoint.h"
#include <architecture/utilities/eigenSupport.h>

FlybyPoint::FlybyPoint() { this->algorithm = FlybyPointAlgorithm(); }

void FlybyPoint::reset(uint64_t currentSimNanos) {
    assert(this->filterInMsg.isLinked());
    this->algorithm.reset();
}

void FlybyPoint::updateState(uint64_t currentSimNanos) {
    auto [r_BN_N, v_BN_N] = this->readRelativeState();
    auto [attMsgBuffer, flybyDiagnosticMsgBuffer] = this->algorithm.updateState(currentSimNanos, r_BN_N, v_BN_N);
    this->attRefOutMsg.write(&attMsgBuffer, this->moduleID, currentSimNanos);
    this->flybyDiagnosticOutMsg.write(&flybyDiagnosticMsgBuffer, this->moduleID, currentSimNanos);
}

std::tuple<Eigen::Vector3d, Eigen::Vector3d> FlybyPoint::readRelativeState() {
    NavTransMsgPayload relativeState = this->filterInMsg();

    Eigen::Vector3d r_BN_N(relativeState.r_BN_N[0], relativeState.r_BN_N[1], relativeState.r_BN_N[2]);
    Eigen::Vector3d v_BN_N(relativeState.v_BN_N[0], relativeState.v_BN_N[1], relativeState.v_BN_N[2]);

    return {r_BN_N, v_BN_N};
}

void FlybyPoint::setTimeBetweenFilterData(double timeBetweenFilterData) {
    this->algorithm.setTimeBetweenFilterData(timeBetweenFilterData);
}

void FlybyPoint::setToleranceForCollinearity(double toleranceForCollinearity) {
    this->algorithm.setToleranceForCollinearity(toleranceForCollinearity);
}

void FlybyPoint::setSignOfOrbitNormalFrameVector(int signOfOrbitNormalFrameVector) {
    this->algorithm.setSignOfOrbitNormalFrameVector(signOfOrbitNormalFrameVector);
}

void FlybyPoint::setMaximumRateThreshold(double maximumRateThreshold) {
    this->algorithm.setMaximumRateThreshold(maximumRateThreshold);
}

void FlybyPoint::setMaximumAccelerationThreshold(double maximumAccelerationThreshold) {
    this->algorithm.setMaximumAccelerationThreshold(maximumAccelerationThreshold);
}

void FlybyPoint::setPositionKnowledgeSigma(double positionKnowledgeStd) {
    this->algorithm.setPositionKnowledgeSigma(positionKnowledgeStd);
}

double FlybyPoint::getTimeBetweenFilterData() const { return this->algorithm.getTimeBetweenFilterData(); }

double FlybyPoint::getToleranceForCollinearity() const { return this->algorithm.getToleranceForCollinearity(); }

int FlybyPoint::getSignOfOrbitNormalFrameVector() const { return this->algorithm.getSignOfOrbitNormalFrameVector(); }

double FlybyPoint::getMaximumAccelerationThreshold() const { return this->algorithm.getMaximumAccelerationThreshold(); }

double FlybyPoint::getMaximumRateThreshold() const { return this->algorithm.getMaximumRateThreshold(); }

double FlybyPoint::getPositionKnowledgeSigma() const { return this->algorithm.getPositionKnowledgeSigma(); }
