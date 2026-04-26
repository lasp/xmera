// SPDX-License-Identifier: ISC
// Copyright (c) 2026, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

#ifndef FILTERING_ALGORITHMS_FLYBY_OD_UKF_ALGORITHM_H
#define FILTERING_ALGORITHMS_FLYBY_OD_UKF_ALGORITHM_H

#include "flybyODuKFTypes.h"

#include <filtering_core/srukf_interface.hpp>
#include <filtering_core/state.hpp>

#include <Eigen/Core>

namespace filtering::flybyODuKF {

// SRUKF Spec: just the typedefs required by `SrukfSpec` concept. The
// actual dynamics function and measurement-model types live with the
// implementation (.cpp); they're plugged into the SrukfInterface via the
// `dynamics` member and the `update<M>(m)` template at the call site.
struct FlybyODuKFSpec {
    using State           = filtering::StateVector<filtering::Position<3>, filtering::Velocity<3>>;
    using ProcessNoiseCov = Eigen::Matrix<double, 6, 6>;
};

// Pure-C++, framework-agnostic flyby OD square-root unscented Kalman
// filter. State is a 6-vector (3-position + 3-velocity). Observation is
// the unit vector from the body toward the target body, expressed in the
// inertial frame.
//
// Mirrors the fp32-fsw-xmera convention (e.g.
// `ConvertStPlatformToBodyAlgorithm`): plain C++ class, Eigen members,
// setters/getters for tunables, mutating step methods, value-returning
// readout methods. No xmera dependencies — the host adapter marshals
// messages in/out of HeadingMeasurement / FilterStateOutput /
// ResidualsOutput.
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

    // ---- Step ---------------------------------------------------------------

    void reset();
    void predict(double dt);
    void update(HeadingMeasurement const& measurement);

    // ---- Readout -------------------------------------------------------------

    FilterStateOutput getState() const;
    ResidualsOutput   getLastResiduals() const;

private:
    // Underlying SRUKF (functional core wrapped in a stateful façade).
    SrukfInterface<FlybyODuKFSpec> filter;

    // Configuration retained on `this` and applied in reset().
    double                      mu                = 0;
    Eigen::Matrix<double, N, N> processNoise      = Eigen::Matrix<double, N, N>::Zero();
    double                      alpha             = 0;
    double                      beta              = 0;
    double                      measNoiseScale    = 1;
    double                      unitConversion    = 1;
    State                       initialState      = {};
    Eigen::Matrix<double, N, N> initialCovariance = Eigen::Matrix<double, N, N>::Identity();

    // Latest residuals captured from update(); reported via getLastResiduals().
    ResidualsOutput lastResiduals = {};

    // Latest update timestamp — used to populate FilterStateOutput::timeTag.
    double currentTime = 0;
};

}  // namespace filtering::flybyODuKF

#endif
