// SPDX-License-Identifier: ISC
// Copyright (c) 2026, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

#ifndef FILTERING_ALGORITHMS_SUNLINE_SR_UKF_SPECS_H
#define FILTERING_ALGORITHMS_SUNLINE_SR_UKF_SPECS_H

#include <filteringCore/state.hpp>

#include <Eigen/Core>

#include <variant>

namespace filtering::sunlineSRuKF {

inline constexpr int MaxCss    = 8;
inline constexpr int BatchSize = 2;

// State: [s_hat (3), omega_BN_B (3), bias (1)].
using SunlineState = filtering::StateVector<
    filtering::Position<3>,
    filtering::Velocity<3>,
    filtering::Bias<1>>;

// ds/dt = s × omega; omega and bias constant under propagation.
struct SunlineDynamics {
    SunlineState operator()(double /*t*/, SunlineState const& state) const {
        Eigen::Vector3d const sHat  = state.get<filtering::Position<3>>();
        Eigen::Vector3d const omega = state.get<filtering::Velocity<3>>();

        SunlineState xDot;
        xDot.set<filtering::Position<3>>(sHat.cross(omega));
        xDot.set<filtering::Velocity<3>>(Eigen::Vector3d::Zero());
        xDot.set<filtering::Bias<1>>(Eigen::Vector<double, 1>::Zero());
        return xDot;
    }
};

struct CssMeasurement {
    double                                timeTag           = 0;
    Eigen::Vector<double, MaxCss>         cssCosValues      = Eigen::Vector<double, MaxCss>::Zero();
    Eigen::Matrix<double, MaxCss, 3>      hMatrix           = Eigen::Matrix<double, MaxCss, 3>::Zero();
    Eigen::Matrix<double, MaxCss, MaxCss> covar             = Eigen::Matrix<double, MaxCss, MaxCss>::Identity();
    int                                   numberOfActiveCss = 0;
    bool                                  valid             = false;
};

struct RateMeasurement {
    double          timeTag    = 0;
    Eigen::Vector3d omega_BN_B = Eigen::Vector3d::Zero();
    Eigen::Matrix3d covar      = Eigen::Matrix3d::Identity();
    bool            valid      = false;
};

using Measurement = std::variant<CssMeasurement, RateMeasurement>;

struct FilterStateOutput {
    static constexpr int N = SunlineState::size;
    Eigen::Matrix<double, N, 1>  state      = Eigen::Matrix<double, N, 1>::Zero();
    Eigen::Matrix<double, N, N>  covariance = Eigen::Matrix<double, N, N>::Zero();
};

struct CssResidualsOutput {
    bool                          valid             = false;
    int                           numberOfActiveCss = 0;
    Eigen::Vector<double, MaxCss> observation       = Eigen::Vector<double, MaxCss>::Zero();
    Eigen::Vector<double, MaxCss> preFit            = Eigen::Vector<double, MaxCss>::Zero();
    Eigen::Vector<double, MaxCss> postFit           = Eigen::Vector<double, MaxCss>::Zero();
};

struct RateResidualsOutput {
    bool            valid       = false;
    Eigen::Vector3d observation = Eigen::Vector3d::Zero();
    Eigen::Vector3d preFit      = Eigen::Vector3d::Zero();
    Eigen::Vector3d postFit     = Eigen::Vector3d::Zero();
};

using Residuals = std::variant<CssResidualsOutput, RateResidualsOutput>;

}  // namespace filtering::sunlineSRuKF

#endif
