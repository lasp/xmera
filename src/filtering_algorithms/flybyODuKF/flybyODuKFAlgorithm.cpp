// SPDX-License-Identifier: ISC
// Copyright (c) 2026, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

#include "flybyODuKFAlgorithm.h"

#include <cmath>
#include <utility>

namespace filtering::flybyODuKF {

namespace {

using State = FlybyODuKFAlgorithm::State;

// Heading measurement model satisfying the Measurement<M, State> concept.
//
// The observation is the (normalized) unit vector from the body toward the
// target in the inertial frame; the predicted observation is the normalized
// estimated position. Both are dimensionless, so no unit conversion is
// applied here (it lives entirely on the state, in the SRUKF core).
struct HeadingMeasurementModel {
    static constexpr int size = 3;

    Eigen::Vector3d observed   = Eigen::Vector3d::Zero();    // normalized rhat_BN_N
    Eigen::Matrix3d measNoise  = Eigen::Matrix3d::Identity();  // measNoiseScale * covarN

    Eigen::Vector3d observation() const { return this->observed; }

    Eigen::Vector3d model(State const& state) const {
        Eigen::Vector3d const r = state.get<filtering::Position<3>>();
        return r / r.norm();
    }

    Eigen::Matrix3d noise() const { return this->measNoise; }

    Eigen::Vector3d subtract(Eigen::Vector3d const& a, Eigen::Vector3d const& b) const {
        return a - b;
    }
};

}  // namespace

// ---- Configuration --------------------------------------------------------

void FlybyODuKFAlgorithm::setMu(double mu) { this->mu = mu; }
double FlybyODuKFAlgorithm::getMu() const { return this->mu; }

void FlybyODuKFAlgorithm::setProcessNoise(Eigen::Matrix<double, N, N> const& processNoise) {
    this->processNoise = processNoise;
}
Eigen::Matrix<double, FlybyODuKFAlgorithm::N, FlybyODuKFAlgorithm::N>
FlybyODuKFAlgorithm::getProcessNoise() const {
    return this->processNoise;
}

void FlybyODuKFAlgorithm::setAlpha(double alpha) { this->alpha = alpha; }
double FlybyODuKFAlgorithm::getAlpha() const { return this->alpha; }

void FlybyODuKFAlgorithm::setBeta(double beta) { this->beta = beta; }
double FlybyODuKFAlgorithm::getBeta() const { return this->beta; }

void FlybyODuKFAlgorithm::setMeasurementNoiseScale(double scale) { this->measNoiseScale = scale; }
double FlybyODuKFAlgorithm::getMeasurementNoiseScale() const { return this->measNoiseScale; }

void FlybyODuKFAlgorithm::setUnitConversion(double conversion) { this->unitConversion = conversion; }
double FlybyODuKFAlgorithm::getUnitConversion() const { return this->unitConversion; }

void FlybyODuKFAlgorithm::setInitialState(State const& s) { this->initialState = s; }
FlybyODuKFAlgorithm::State FlybyODuKFAlgorithm::getInitialState() const { return this->initialState; }

void FlybyODuKFAlgorithm::setInitialCovariance(Eigen::Matrix<double, N, N> const& P) {
    this->initialCovariance = P;
}
Eigen::Matrix<double, FlybyODuKFAlgorithm::N, FlybyODuKFAlgorithm::N>
FlybyODuKFAlgorithm::getInitialCovariance() const {
    return this->initialCovariance;
}

// ---- Step -----------------------------------------------------------------

void FlybyODuKFAlgorithm::reset() {
    this->filter.setAlpha(this->alpha);
    this->filter.setBeta(this->beta);
    this->filter.setMeasurementNoiseScale(this->measNoiseScale);
    this->filter.setUnitConversion(this->unitConversion);
    this->filter.setProcessNoise(this->processNoise);
    this->filter.setInitialMean(this->initialState);
    this->filter.setInitialCovariance(this->initialCovariance);

    // Two-body point-mass gravity. The SRUKF core works in the converted unit
    // system (mean is scaled by unitConversion in reset()), so mu — input in
    // m^3/s^2 — is scaled by unitConversion^3 to match.
    double const centralBody = this->mu * std::pow(this->unitConversion, 3);
    this->filter.dynamics = [centralBody](double /*t*/, State const& state) -> State {
        Eigen::Vector3d const r = state.get<filtering::Position<3>>();
        Eigen::Vector3d const v = state.get<filtering::Velocity<3>>();

        State xDot;
        xDot.set<filtering::Position<3>>(v);
        xDot.set<filtering::Velocity<3>>(-centralBody / std::pow(r.norm(), 3) * r);
        return xDot;
    };

    this->filter.reset();
    this->measurements.clear();
    this->currentTime = 0;
}

void FlybyODuKFAlgorithm::enqueueMeasurement(double timeTag, HeadingMeasurement measurement) {
    this->measurements.enqueue(timeTag, std::move(measurement));
}

void FlybyODuKFAlgorithm::update(double previousSeconds, double currentSeconds) {
    // applyToFilter walks the queue in time order, calling timeUpdate() for the
    // intervening predict steps and measurementUpdate() at each measurement,
    // then a final timeUpdate() to currentSeconds. It empties the queue as it
    // consumes each measurement.
    this->measurements.applyToFilter(*this, previousSeconds, currentSeconds);
}

void FlybyODuKFAlgorithm::timeUpdate(double dt) {
    this->filter.predict(dt);
    this->currentTime += dt;
}

void FlybyODuKFAlgorithm::measurementUpdate(HeadingMeasurement const& measurement) {
    HeadingMeasurementModel model;
    model.observed  = measurement.rhat_BN_N.normalized();
    model.measNoise = this->measNoiseScale * measurement.covarN;

    auto const result = this->filter.update(model);

    this->lastResiduals.valid       = measurement.valid;
    this->lastResiduals.observation = model.observed;
    this->lastResiduals.preFit      = result.preFit;
    this->lastResiduals.postFit     = result.postFit;

    this->currentTime = measurement.timeTag;
}

// ---- Readout --------------------------------------------------------------

FilterStateOutput FlybyODuKFAlgorithm::getState() const {
    FilterStateOutput out;
    out.timeTag = this->currentTime;
    // Undo the internal unit conversion: state scales by 1/unitConversion,
    // and the square-root covariance (sqrt of a 1/unitConversion^2 quantity)
    // by 1/unitConversion as well.
    out.state     = this->filter.getMean().raw() / this->unitConversion;
    out.sqrtCovar = this->filter.getSqrtCovar() / this->unitConversion;
    return out;
}

ResidualsOutput FlybyODuKFAlgorithm::getLastResiduals() const { return this->lastResiduals; }

}  // namespace filtering::flybyODuKF
