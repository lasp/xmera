// SPDX-License-Identifier: ISC
// Copyright (c) 2026, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

#include "sunlineSRuKFAlgorithm.h"

#include <algorithm>
#include <cassert>
#include <variant>

namespace filtering::sunlineSRuKF {

namespace {

using State = SunlineSRuKFAlgorithm::State;

/*! CSS observation = bias * H * s_hat. Satisfies the Measurement
 *  concept for use inside the SRuKF. */
struct CssMeasurementModel {
    static constexpr int size = MaxCss;

    Eigen::Vector<double, MaxCss>         observed  = Eigen::Vector<double, MaxCss>::Zero();
    Eigen::Matrix<double, MaxCss, 3>      hMatrix   = Eigen::Matrix<double, MaxCss, 3>::Zero();
    Eigen::Matrix<double, MaxCss, MaxCss> measNoise = Eigen::Matrix<double, MaxCss, MaxCss>::Identity();

    Eigen::Vector<double, MaxCss> observation() const { return this->observed; }
    Eigen::Vector<double, MaxCss> model(State const& state) const {
        double const bias = state.get<filtering::Bias<1>>()(0);
        Eigen::Vector3d const sHat = state.get<filtering::Position<3>>();
        return bias * (this->hMatrix * sHat);
    }
    Eigen::Matrix<double, MaxCss, MaxCss> noise() const { return this->measNoise; }
    Eigen::Vector<double, MaxCss> subtract(
        Eigen::Vector<double, MaxCss> const& a,
        Eigen::Vector<double, MaxCss> const& b
    ) const { return a - b; }
};

/*! Predicted gyro observation = omega_BN_B. Satisfies the Measurement concept. */
struct RateMeasurementModel {
    static constexpr int size = 3;

    Eigen::Vector3d observed  = Eigen::Vector3d::Zero();
    Eigen::Matrix3d measNoise = Eigen::Matrix3d::Identity();

    Eigen::Vector3d observation() const { return this->observed; }
    Eigen::Vector3d model(State const& state) const { return state.get<filtering::Velocity<3>>(); }
    Eigen::Matrix3d noise() const { return this->measNoise; }
    Eigen::Vector3d subtract(Eigen::Vector3d const& a, Eigen::Vector3d const& b) const { return a - b; }
};

static_assert(filtering::Measurement<CssMeasurementModel, State>);
static_assert(filtering::Measurement<RateMeasurementModel, State>);

}  // namespace

/*! Reset the filter states, provide dynamics, and clear
 *  any state carried from a previous run.
 *  @return void */
void SunlineSRuKFAlgorithm::reset() {
    this->srukf.setAlpha(this->alpha);
    this->srukf.setBeta(this->beta);
    this->srukf.setProcessNoise(this->processNoise);
    this->srukf.setInitialState(this->initialState);
    this->srukf.setInitialCovariance(this->initialCovariance);
    this->srukf.dynamics = SunlineDynamics{};

    this->srukf.reset();
    this->measurements.clear();
    this->lastCssResiduals  = CssResidualsOutput{};
    this->lastRateResiduals = RateResidualsOutput{};
}


/*! Main entrypoint. Enqueues whichever measurements are present, empties
 *  the queue through the SRuKF, then sanitizes the state.
 *  @return Snapshot of post-update filter state and residuals.
 *  @param currentSeconds [s] simulation time the filter is advancing to
 *  @param cssData        [-] CSS array reading + validity flag
 *  @param rateData       [-] gyro reading + validity flag */
SunlineSRuKFOutput SunlineSRuKFAlgorithm::update(double currentSeconds,
                                                  CssData const& cssData,
                                                  RateData const& rateData) {
    this->lastCssResiduals.valid  = false;
    this->lastRateResiduals.valid = false;

    if (cssData.timeTag > 0) {
        this->measurements.enqueue(cssData.timeTag, this->packCssMeasurement(cssData));
    }
    if (rateData.timeTag > 0) {
        this->measurements.enqueue(rateData.timeTag, this->packRateMeasurement(rateData));
    }
    apply_sequential(this->measurements, *this, currentSeconds);
    this->srukf.setState(this->regularize(this->srukf.getState()));
    this->srukf.setStateLastMeasurement(this->regularize(this->srukf.getStateAtLastMeasurement()));

    return SunlineSRuKFOutput{
        .filterState   = this->getFilterOutput(),
        .cssResiduals  = this->lastCssResiduals,
        .rateResiduals = this->lastRateResiduals,
    };
}

/*! Propagate the state from the last-measurement anchor by dt.
 *  @return void
 *  @param dt [s] elapsed time since the last measurement */
void SunlineSRuKFAlgorithm::timeUpdate(double dt) { this->srukf.timeUpdate(dt); }

/*! Fold a single measurement into the filter; dispatches per-kind.
 *  @return void
 *  @param measurement [-] CSS- or rate-kind measurement to apply */
void SunlineSRuKFAlgorithm::measurementUpdate(Measurement const& measurement) {
    std::visit([this](auto const& meas) { this->applyMeasurement(meas); }, measurement);
}

/*! Apply a CSS measurement and record its residuals.
 *  @return void
 *  @param measurement [-] packed CSS measurement (cosValues, H, noise) */
void SunlineSRuKFAlgorithm::applyMeasurement(CssMeasurement const& measurement) {
    CssMeasurementModel model;
    model.observed  = measurement.cssCosValues;
    model.hMatrix   = measurement.hMatrix;
    model.measNoise = measurement.covar;

    auto const result = this->srukf.measurementUpdate(model);

    this->lastCssResiduals.valid             = measurement.valid;
    this->lastCssResiduals.numberOfActiveCss = measurement.numberOfActiveCss;
    this->lastCssResiduals.observation       = model.observed;
    this->lastCssResiduals.preFit            = result.preFit;
    this->lastCssResiduals.postFit           = result.postFit;
}

/*! Apply a rate measurement and record its residuals.
 *  @return void
 *  @param measurement [-] packed rate measurement (omega, noise) */
void SunlineSRuKFAlgorithm::applyMeasurement(RateMeasurement const& measurement) {
    RateMeasurementModel model;
    model.observed  = measurement.omega_BN_B;
    model.measNoise = measurement.covar;

    auto const result = this->srukf.measurementUpdate(model);

    this->lastRateResiduals.valid       = measurement.valid;
    this->lastRateResiduals.observation = model.observed;
    this->lastRateResiduals.preFit      = result.preFit;
    this->lastRateResiduals.postFit     = result.postFit;
}

/*! Pack CSS readings into a CssMeasurement: active rows
 *  (cosValue > sensorUseThresh) populate the first `active` slots; H rows
 *  carry CBias * nHat vectors.
 *  @return CssMeasurement (valid = active > 0)
 *  @param cssData [-] raw CSS cos-values and time tag */
CssMeasurement SunlineSRuKFAlgorithm::packCssMeasurement(CssData const& cssData) {
    CssMeasurement packed;
    packed.timeTag = cssData.timeTag;
    packed.covar   = (this->cssMeasNoiseStd * this->cssMeasNoiseStd)
                     * Eigen::Matrix<double, MaxCss, MaxCss>::Identity();

    int active = 0;
    for (int i = 0; i < this->numberOfCss && active < MaxCss; ++i) {
        if (cssData.cosValues(i) <= this->sensorUseThresh) continue;
        packed.cssCosValues(active) = cssData.cosValues(i);
        packed.hMatrix.row(active)  = this->cssCBias(i) * this->cssNHat.row(i);
        active += 1;
    }
    packed.numberOfActiveCss = active;
    packed.valid             = active > 0;
    return packed;
}

/*! Pack a raw gyro reading into a RateMeasurement with diagonal noise covar.
 *  @return RateMeasurement (always valid)
 *  @param rateData [-] raw rate vector and time tag */
RateMeasurement SunlineSRuKFAlgorithm::packRateMeasurement(RateData const& rateData) {
    RateMeasurement packed;
    packed.timeTag    = rateData.timeTag;
    packed.omega_BN_B = rateData.rate;
    packed.covar      = (this->gyroMeasNoiseStd * this->gyroMeasNoiseStd)
                        * Eigen::Matrix3d::Identity();
    packed.valid      = true;
    return packed;
}

/*! Renormalize the heading and clamp the bias to its configured bounds.
 *  @return sanitized copy of State
 *  @param state [-] state to regularize */
SunlineSRuKFAlgorithm::State SunlineSRuKFAlgorithm::regularize(State const& state) const {
    State outputState = state;
    outputState.set<filtering::Position<3>>(outputState.get<filtering::Position<3>>().normalized());
    Eigen::Vector<double, 1> biasVec;
    biasVec(0) = std::clamp(outputState.get<filtering::Bias<1>>()(0),
                            this->biasLowerBound, this->biasUpperBound);
    outputState.set<filtering::Bias<1>>(biasVec);
    return outputState;
}

/*! Bundle the SRuKF's current state and covariance into the output POD.
 *  @return FilterStateOutput */
FilterStateOutput SunlineSRuKFAlgorithm::getFilterOutput() const {
    FilterStateOutput filterOutput;
    filterOutput.state      = this->srukf.getState().raw();
    filterOutput.covariance = this->srukf.getCovariance();
    return filterOutput;
}

/*! @return const reference to the latest CSS residuals snapshot */
CssResidualsOutput const& SunlineSRuKFAlgorithm::getLastCssResiduals() const { return this->lastCssResiduals; }
/*! @return const reference to the latest rate residuals snapshot */
RateResidualsOutput const& SunlineSRuKFAlgorithm::getLastRateResiduals() const { return this->lastRateResiduals; }

/*! @return current filter state (post-sanitization) */
SunlineSRuKFAlgorithm::State SunlineSRuKFAlgorithm::getState() const { return this->srukf.getState(); }
/*! @return current full covariance */
Eigen::Matrix<double, SunlineSRuKFAlgorithm::N, SunlineSRuKFAlgorithm::N>
SunlineSRuKFAlgorithm::getCovariance() const { return this->srukf.getCovariance(); }

/*! Set the process noise.
 *  @param newProcessNoise [-] N x N process noise covariance */
void SunlineSRuKFAlgorithm::setProcessNoise(Eigen::Matrix<double, N, N> const& newProcessNoise) {
    this->processNoise = newProcessNoise;
}
/*! @return current process noise */
Eigen::Matrix<double, SunlineSRuKFAlgorithm::N, SunlineSRuKFAlgorithm::N>
SunlineSRuKFAlgorithm::getProcessNoise() const { return this->processNoise; }

/*! Set the UKF alpha tunable.
 *  @param newAlpha [-] sigma-point spread */
void   SunlineSRuKFAlgorithm::setAlpha(double newAlpha) { this->alpha = newAlpha; }
/*! @return current UKF alpha */
double SunlineSRuKFAlgorithm::getAlpha() const          { return this->alpha; }

/*! Set the UKF beta tunable.
 *  @param newBeta [-] prior-knowledge constant */
void   SunlineSRuKFAlgorithm::setBeta(double newBeta) { this->beta = newBeta; }
/*! @return current UKF beta */
double SunlineSRuKFAlgorithm::getBeta() const         { return this->beta; }

/*! Set the initial state seed (consumed by reset()).
 *  @param newInitialState [-] N-element state */
void SunlineSRuKFAlgorithm::setInitialState(State const& newInitialState) {
    this->initialState = newInitialState;
}
/*! @return current initial state seed */
SunlineSRuKFAlgorithm::State SunlineSRuKFAlgorithm::getInitialState() const { return this->initialState; }

/*! Set the initial covariance seed (consumed by reset()).
 *  @param newInitialCovariance [-] N x N covariance */
void SunlineSRuKFAlgorithm::setInitialCovariance(Eigen::Matrix<double, N, N> const& newInitialCovariance) {
    this->initialCovariance = newInitialCovariance;
}
/*! @return current initial covariance seed */
Eigen::Matrix<double, SunlineSRuKFAlgorithm::N, SunlineSRuKFAlgorithm::N>
SunlineSRuKFAlgorithm::getInitialCovariance() const { return this->initialCovariance; }

/*! Set the lower clamp on the CSS bias state.
 *  @param lowerBound [-] bias lower bound */
void   SunlineSRuKFAlgorithm::setBiasLowerBound(double lowerBound) { this->biasLowerBound = lowerBound; }
/*! @return current bias lower bound */
double SunlineSRuKFAlgorithm::getBiasLowerBound() const            { return this->biasLowerBound; }
/*! Set the upper clamp on the CSS bias state.
 *  @param upperBound [-] bias upper bound */
void   SunlineSRuKFAlgorithm::setBiasUpperBound(double upperBound) { this->biasUpperBound = upperBound; }
/*! @return current bias upper bound */
double SunlineSRuKFAlgorithm::getBiasUpperBound() const            { return this->biasUpperBound; }

/*! Set the per-sensor CSS unit vectors (body frame).
 *  @param nHat [-] MaxCss x 3 matrix; only the first `numberOfCss` rows matter */
void SunlineSRuKFAlgorithm::setCssNHat(Eigen::Matrix<double, MaxCss, 3> const& nHat) { this->cssNHat = nHat; }
/*! @return CSS unit-vector matrix */
Eigen::Matrix<double, MaxCss, 3> SunlineSRuKFAlgorithm::getCssNHat() const { return this->cssNHat; }

/*! Set the per-sensor CSS calibration biases.
 *  @param cBias [-] MaxCss-vector; only the first `numberOfCss` entries matter */
void SunlineSRuKFAlgorithm::setCssCBias(Eigen::Vector<double, MaxCss> const& cBias) { this->cssCBias = cBias; }
/*! @return CSS calibration-bias vector */
Eigen::Vector<double, MaxCss> SunlineSRuKFAlgorithm::getCssCBias() const { return this->cssCBias; }

/*! Set the count of configured CSS sensors.
 *  @param count [-] number of CSS sensors; must satisfy 0 <= count <= MaxCss */
void SunlineSRuKFAlgorithm::setNumberOfCss(int count) {
    assert(count >= 0 && count <= MaxCss);
    this->numberOfCss = count;
}
/*! @return number of configured CSS sensors */
int SunlineSRuKFAlgorithm::getNumberOfCss() const { return this->numberOfCss; }

/*! Set the activation threshold on CSS cos-values.
 *  @param threshold [-] minimum cosValue to count a sensor as active */
void   SunlineSRuKFAlgorithm::setSensorThreshold(double threshold) { this->sensorUseThresh = threshold; }
/*! @return current activation threshold */
double SunlineSRuKFAlgorithm::getSensorThreshold() const           { return this->sensorUseThresh; }

/*! Set the CSS measurement noise std.
 *  @param noiseStd [-] noise std (sigma) for each CSS observation */
void   SunlineSRuKFAlgorithm::setCssMeasurementNoiseStd(double noiseStd) { this->cssMeasNoiseStd = noiseStd; }
/*! @return current CSS noise std */
double SunlineSRuKFAlgorithm::getCssMeasurementNoiseStd() const          { return this->cssMeasNoiseStd; }

/*! Set the gyro measurement noise std.
 *  @param noiseStd [rad/s] noise std (sigma) for each rate observation */
void   SunlineSRuKFAlgorithm::setGyroMeasurementNoiseStd(double noiseStd) { this->gyroMeasNoiseStd = noiseStd; }
/*! @return current gyro noise std */
double SunlineSRuKFAlgorithm::getGyroMeasurementNoiseStd() const          { return this->gyroMeasNoiseStd; }

}  // namespace filtering::sunlineSRuKF
