// SPDX-License-Identifier: ISC
// Copyright (c) 2026, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

#include "sunlineSRuKF.h"

#include "sunlineSRuKFAlgorithm.h"
#include "sunlineSRuKFSpecs.h"

#include <architecture/utilities/eigenSupport.h>
#include <architecture/utilities/macroDefinitions.h>

#include <Eigen/Core>

#include <cassert>

using filtering::sunlineSRuKF::CssData;
using filtering::sunlineSRuKF::MaxCss;
using filtering::sunlineSRuKF::RateData;
using filtering::sunlineSRuKF::SunlineSRuKFAlgorithm;
using filtering::sunlineSRuKF::SunlineSRuKFOutput;
using filtering::sunlineSRuKF::SunlineState;

SunlineSRuKF::SunlineSRuKF() : algorithm(std::make_unique<SunlineSRuKFAlgorithm>()) {}

SunlineSRuKF::~SunlineSRuKF() = default;

/*! Latch the CSS geometry from cssConfigInMsg into the algorithm, then reset
 *  the algorithm and the per-channel freshness trackers.
 *  @return void
 *  @param currentSimNanos [ns] sim time at which reset was called */
void SunlineSRuKF::reset(uint64_t currentSimNanos) {
    assert(this->navAttInMsg.isLinked());
    assert(this->cssDataInMsg.isLinked());
    assert(this->cssConfigInMsg.isLinked());

    auto const cssConfig = this->cssConfigInMsg();
    int const numCss = static_cast<int>(cssConfig.nCSS);
    Eigen::Matrix<double, MaxCss, 3> nHat  = Eigen::Matrix<double, MaxCss, 3>::Zero();
    Eigen::Vector<double, MaxCss>    cBias = Eigen::Vector<double, MaxCss>::Zero();
    for (int i = 0; i < numCss; ++i) {
        cBias(i) = cssConfig.cssVals[i].CBias;
        for (int j = 0; j < 3; ++j) {
            nHat(i, j) = cssConfig.cssVals[i].nHat_B[j];
        }
    }
    this->algorithm->setCssNHat(nHat);
    this->algorithm->setCssCBias(cBias);
    this->algorithm->setNumberOfCss(numCss);  // asserts numCss in [0, MaxCss]

    this->algorithm->reset();
    this->lastNavAttTimeTag = 0;
    this->lastCssTimeTag    = 0;
}

/*! Read NavAtt and CSS messages, call algorithm update, and
 *  write the output state and residuals.
 *  @return void
 *  @param currentSimNanos [ns] sim time the filter is advancing to */
void SunlineSRuKF::updateState(uint64_t currentSimNanos) {
    double const currentSeconds = static_cast<double>(currentSimNanos) * NANO2SEC;

    RateData rateData{};
    CssData  cssData{};

    if (auto const navMsgPayload = this->navAttInMsg(); navMsgPayload.timeTag > this->lastNavAttTimeTag) {
        rateData.timeTag = navMsgPayload.timeTag;
        rateData.rate    = cArrayToEigenVector(navMsgPayload.omega_BN_B);
        this->lastNavAttTimeTag = navMsgPayload.timeTag;
    }

    if (auto const [timeTag, CosValue] = this->cssDataInMsg();
        timeTag > this->lastCssTimeTag) {
        Eigen::Vector<double, MaxCss> sensorMeasurements{};
        for (int i = 0; i < this->algorithm->getNumberOfCss(); ++i) {
            sensorMeasurements[i] = CosValue[i];
        }
        cssData.timeTag   = timeTag;
        cssData.cosValues = sensorMeasurements;
        this->lastCssTimeTag = timeTag;
    }

    SunlineSRuKFOutput const filterOutput =
        this->algorithm->update(currentSeconds, cssData, rateData);
    this->writeOutputMessages(currentSimNanos, filterOutput);
}

/*! Write the algorithm's output to the four xmera output messages.
 *  @return void
 *  @param currentSimNanos [ns] sim time provided to the outgoing messages
 *  @param filterOutput    [-]  filter data returned by algorithm */
void SunlineSRuKF::writeOutputMessages(uint64_t currentSimNanos,
                                       SunlineSRuKFOutput const& filterOutput) {
    NavAttMsgPayload          navAttBuf{};
    FilterMsgPayload          filterBuf{};
    FilterResidualsMsgPayload gyroResBuf{};
    FilterResidualsMsgPayload cssResBuf{};

    double const timeTag = static_cast<double>(currentSimNanos) * NANO2SEC;

    eigenMatrixXToCArray(filterOutput.filterState.state.head<3>().eval(), navAttBuf.vehSunPntBdy);

    filterBuf.timeTag        = timeTag;
    filterBuf.numberOfStates = SunlineSRuKFAlgorithm::N;
    eigenMatrixXToCArray(filterOutput.filterState.state,      filterBuf.state);
    eigenMatrixXToCArray(filterOutput.filterState.covariance, filterBuf.covar);

    if (filterOutput.cssResiduals.valid) {
        cssResBuf.timeTag              = timeTag;
        cssResBuf.valid                = true;
        cssResBuf.numberOfObservations = 1;
        cssResBuf.sizeOfObservations   = filterOutput.cssResiduals.numberOfActiveCss;
        eigenMatrixXToCArray(filterOutput.cssResiduals.observation, cssResBuf.observation);
        eigenMatrixXToCArray(filterOutput.cssResiduals.preFit,      cssResBuf.preFits);
        eigenMatrixXToCArray(filterOutput.cssResiduals.postFit,     cssResBuf.postFits);
    }
    if (filterOutput.rateResiduals.valid) {
        gyroResBuf.timeTag              = timeTag;
        gyroResBuf.valid                = true;
        gyroResBuf.numberOfObservations = 1;
        gyroResBuf.sizeOfObservations   = 3;
        eigenMatrixXToCArray(filterOutput.rateResiduals.observation, gyroResBuf.observation);
        eigenMatrixXToCArray(filterOutput.rateResiduals.preFit,      gyroResBuf.preFits);
        eigenMatrixXToCArray(filterOutput.rateResiduals.postFit,     gyroResBuf.postFits);
    }

    this->navAttOutMsg.write(&navAttBuf,         this->moduleID, currentSimNanos);
    this->filterOutMsg.write(&filterBuf,         this->moduleID, currentSimNanos);
    this->filterGyroResOutMsg.write(&gyroResBuf, this->moduleID, currentSimNanos);
    this->filterCssResOutMsg.write(&cssResBuf,   this->moduleID, currentSimNanos);
}

/*! Set the UKF alpha parameter.
 *  @return void
 *  @param newAlpha [-] sigma-point spread */
void   SunlineSRuKF::setAlpha(double newAlpha) { this->algorithm->setAlpha(newAlpha); }
/*! @return current UKF alpha */
double SunlineSRuKF::getAlpha() const          { return this->algorithm->getAlpha(); }

/*! Set the UKF beta parameter.
 *  @return void
 *  @param newBeta [-] prior-knowledge constant */
void   SunlineSRuKF::setBeta(double newBeta) { this->algorithm->setBeta(newBeta); }
/*! @return current UKF beta */
double SunlineSRuKF::getBeta() const         { return this->algorithm->getBeta(); }

/*! Set the filter process noise.
 *  @return void
 *  @param newProcessNoise [-] N x N process noise covariance */
void SunlineSRuKF::setProcessNoise(Eigen::MatrixXd const& newProcessNoise) {
    using Mat = Eigen::Matrix<double, SunlineSRuKFAlgorithm::N, SunlineSRuKFAlgorithm::N>;
    this->algorithm->setProcessNoise(Mat(newProcessNoise));
}
/*! @return current filter process noise */
Eigen::MatrixXd SunlineSRuKF::getProcessNoise() const { return this->algorithm->getProcessNoise(); }

/*! Set the initial state.
 *  @return void
 *  @param newInitialState [-] N-element flat state vector */
void SunlineSRuKF::setInitialState(Eigen::VectorXd const& newInitialState) {
    using Vec = Eigen::Vector<double, SunlineSRuKFAlgorithm::N>;
    this->algorithm->setInitialState(SunlineState{Vec(newInitialState)});
}
/*! @return current initial state */
Eigen::VectorXd SunlineSRuKF::getInitialState() const { return this->algorithm->getInitialState().raw(); }

/*! Set the initial covariance.
 *  @return void
 *  @param newInitialCovariance [-] N x N covariance */
void SunlineSRuKF::setInitialCovariance(Eigen::MatrixXd const& newInitialCovariance) {
    using Mat = Eigen::Matrix<double, SunlineSRuKFAlgorithm::N, SunlineSRuKFAlgorithm::N>;
    this->algorithm->setInitialCovariance(Mat(newInitialCovariance));
}
/*! @return current initial covariance */
Eigen::MatrixXd SunlineSRuKF::getInitialCovariance() const { return this->algorithm->getInitialCovariance(); }

/*! Set the lower bound on the CSS bias state.
 *  @return void
 *  @param lowerBound [-] bias lower bound */
void   SunlineSRuKF::setBiasLowerBound(double lowerBound) { this->algorithm->setBiasLowerBound(lowerBound); }
/*! @return current bias lower bound */
double SunlineSRuKF::getBiasLowerBound() const            { return this->algorithm->getBiasLowerBound(); }

/*! Set the upper bound on the CSS bias state.
 *  @return void
 *  @param upperBound [-] bias upper bound */
void   SunlineSRuKF::setBiasUpperBound(double upperBound) { this->algorithm->setBiasUpperBound(upperBound); }
/*! @return current bias upper bound */
double SunlineSRuKF::getBiasUpperBound() const            { return this->algorithm->getBiasUpperBound(); }

/*! Set the CSS measurement noise std.
 *  @return void
 *  @param noiseStd [-] noise std for each CSS observation */
void   SunlineSRuKF::setCssMeasurementNoiseStd(double noiseStd)  { this->algorithm->setCssMeasurementNoiseStd(noiseStd); }
/*! @return current CSS noise std */
double SunlineSRuKF::getCssMeasurementNoiseStd() const           { return this->algorithm->getCssMeasurementNoiseStd(); }

/*! Set the gyro measurement noise std.
 *  @return void
 *  @param noiseStd [rad/s] noise std for each rate observation */
void   SunlineSRuKF::setGyroMeasurementNoiseStd(double noiseStd) { this->algorithm->setGyroMeasurementNoiseStd(noiseStd); }
/*! @return current gyro noise std */
double SunlineSRuKF::getGyroMeasurementNoiseStd() const          { return this->algorithm->getGyroMeasurementNoiseStd(); }

/*! Set the CSS measurement threshold.
 *  @return void
 *  @param threshold [-] minimum cosValue to count a sensor as active */
void   SunlineSRuKF::setSensorThreshold(double threshold) { this->algorithm->setSensorThreshold(threshold); }
/*! @return current CSS activation threshold */
double SunlineSRuKF::getSensorThreshold() const           { return this->algorithm->getSensorThreshold(); }
