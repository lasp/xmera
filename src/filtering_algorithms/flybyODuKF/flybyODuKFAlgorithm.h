// SPDX-License-Identifier: ISC
// Copyright (c) 2026, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

#ifndef FILTERING_ALGORITHMS_FLYBY_OD_UKF_ALGORITHM_H
#define FILTERING_ALGORITHMS_FLYBY_OD_UKF_ALGORITHM_H

#include "flybyODuKFTypes.h"

#include <filtering_core/kalman_filter.hpp>
#include <filtering_core/srukf_interface.hpp>
#include <filtering_core/state.hpp>

#include <Eigen/Core>

namespace filtering::flybyODuKF {

// Flyby OD square-root unscented Kalman filter. State is a 6-vector
// (3-position + 3-velocity); the observation is the unit vector from the
// body toward the target body, expressed in the inertial frame.
//
// Single composition root: owns the SRUKF engine, the retained
// configuration, the dynamics closure, the internal HeadingMeasurementModel,
// and a bounded measurement_queue. Satisfies
// `SequentialFilter<FlybyODuKFAlgorithm, HeadingMeasurement>` directly via
// the public `timeUpdate` / `measurementUpdate` pair, so a host can either
// drive the filter step-by-step or queue measurements and call
// `update(t0, t1)` to drain the queue through `apply_sequential`.
//
// No xmera dependencies — the host adapter marshals messages in/out of
// HeadingMeasurement / FilterStateOutput / ResidualsOutput.
class FlybyODuKFAlgorithm {
public:
    using State = filtering::StateVector<filtering::Position<3>, filtering::Velocity<3>>;
    static constexpr int N = State::size;  // 6

    // ---- Configuration (call before reset) -----------------------------------

    void setMu(double mu);
    double getMu() const;

    void setProcessNoise(Eigen::Matrix<double, N, N> const& processNoise);
    Eigen::Matrix<double, N, N> getProcessNoise() const;

    void setAlpha(double alpha);
    double getAlpha() const;

    void setBeta(double beta);
    double getBeta() const;

    void setMeasurementNoiseScale(double scale);
    double getMeasurementNoiseScale() const;

    void setUnitConversion(double conversion);
    double getUnitConversion() const;

    void setInitialState(State const& s);
    State getInitialState() const;

    void setInitialCovariance(Eigen::Matrix<double, N, N> const& P);
    Eigen::Matrix<double, N, N> getInitialCovariance() const;

    // ---- Lifecycle + SequentialFilter<…, HeadingMeasurement> -----------------

    void reset();
    void timeUpdate(double dt);
    void measurementUpdate(HeadingMeasurement const& measurement);

    // ---- Queue-driven entry --------------------------------------------------

    // Queue a measurement to be folded in on the next update().
    void enqueueMeasurement(double timeTag, HeadingMeasurement measurement);

    // Drain the queue over [previousSeconds, currentSeconds] via
    // `apply_sequential` — interleaves timeUpdate/measurementUpdate against
    // *this in chronological order, ending at currentSeconds.
    void update(double previousSeconds, double currentSeconds);

    // ---- POD-packaged readouts (SI) ------------------------------------------

    FilterStateOutput getState() const;
    ResidualsOutput   getLastResiduals() const;

    // Raw readouts in filter-internal units (mainly for tests).
    State getMean() const;
    Eigen::Matrix<double, N, N> getSqrtCovar() const;
    double getCurrentTime() const;

private:
    // Underlying SRUKF (functional core wrapped in a stateful façade).
    SrukfInterface<State> srukf;

    // Bounded queue of pending measurements, drained in time order by update().
    filtering::measurement_queue<HeadingMeasurement, 1> measurements;

    // Configuration retained on `this` and applied in reset().
    double                      mu                = 0;
    Eigen::Matrix<double, N, N> processNoise      = Eigen::Matrix<double, N, N>::Zero();
    double                      alpha             = 0;
    double                      beta              = 0;
    double                      measNoiseScale    = 1;
    double                      unitConversion    = 1;
    State                       initialState      = {};
    Eigen::Matrix<double, N, N> initialCovariance = Eigen::Matrix<double, N, N>::Identity();

    // Latest residuals captured from measurementUpdate().
    ResidualsOutput lastResiduals = {};

    // Latest update timestamp.
    double currentTime = 0;
};

}  // namespace filtering::flybyODuKF

#endif
