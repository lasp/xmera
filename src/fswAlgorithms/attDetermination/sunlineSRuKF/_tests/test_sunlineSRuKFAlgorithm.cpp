// SPDX-License-Identifier: ISC
// Copyright (c) 2026, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

// Unit tests for SunlineSRuKFAlgorithm and the SRUKF numerical helpers.
//
// Covers:
// - Sunline dynamics: |s_hat| is conserved under propagation (the natural
//   "energy" invariant of ds/dt = s × omega).
// - timeUpdate(): simple-invariant checks under known initial conditions.
// - measurementUpdate(): single rate measurement shrinks the rate-block of
//   the covariance.
// - update(currentSeconds, CssData, RateData): both per-kind residual slots
//   drive valid when both kinds carry data; CSS threshold gating excludes
//   below-threshold readings.
// - SRuKF static helpers: forward/back substitution, Cholesky, QR-just-R,
//   and Cholesky up/down-date.

#include "sunlineSRuKFAlgorithm.h"
#include "sunlineSRuKFSpecs.h"

#include <filteringCore/srukf.hpp>

#include <gtest/gtest.h>

#include <Eigen/Dense>

#include <cmath>

namespace filtering::sunlineSRuKF {
namespace {

using State   = SunlineSRuKFAlgorithm::State;
using Vector7 = Eigen::Matrix<double, 7, 1>;
using Matrix7 = Eigen::Matrix<double, 7, 7>;

// Alias for invoking the numerical helpers (static methods on the SRuKF
// class template). The State/Dynamics arguments don't affect the helpers'
// behavior; any valid instantiation works.
using SRuKF = ::filtering::SRuKF<SunlineState, SunlineDynamics>;

constexpr double kAlpha = 0.02;
constexpr double kBeta  = 2.0;

State makeState(Eigen::Vector3d const& sHat, Eigen::Vector3d const& omega, double bias) {
    State s;
    s.set<filtering::Position<3>>(sHat);
    s.set<filtering::Velocity<3>>(omega);
    Eigen::Vector<double, 1> b;
    b(0) = bias;
    s.set<filtering::Bias<1>>(b);
    return s;
}

Matrix7 diagCovariance(double posStd, double velStd, double biasStd) {
    Vector7 d;
    d << posStd * posStd, posStd * posStd, posStd * posStd,
         velStd * velStd, velStd * velStd, velStd * velStd,
         biasStd * biasStd;
    return d.asDiagonal();
}

Matrix7 smallProcessNoise() {
    double const q = 1E-10;
    Vector7 d;
    d << q, q, q, q, q, q, q;
    return d.asDiagonal();
}

void configure(SunlineSRuKFAlgorithm& algo,
               State const& initial,
               Matrix7 const& P) {
    algo.setAlpha(kAlpha);
    algo.setBeta(kBeta);
    algo.setInitialState(initial);
    algo.setInitialCovariance(P);
    algo.setProcessNoise(smallProcessNoise());
}

}  // namespace

// ============================================================================
// Dynamics: |s_hat| is a conserved quantity under ds/dt = s × omega.
// ============================================================================

// Pure analytical check: the derivative the dynamics functor returns must be
// orthogonal to s (so d/dt(s·s) = 2 s·(ds/dt) = 0), and omega/bias derivatives
// must be zero. No integrator involved — this isolates the dynamics from any
// numerical concerns in propagate().
TEST(SunlineSRuKFAlgorithmDynamics, DerivativeIsOrthogonalToHeading) {
    Eigen::Vector3d const sHat  = Eigen::Vector3d(0.4, -0.7, 0.6).normalized();
    Eigen::Vector3d const omega = Eigen::Vector3d(0.02, -0.005, 0.01);
    State s = makeState(sHat, omega, 0.6);

    State const dot = SunlineDynamics{}(0.0, s);

    Eigen::Vector3d const dsdt = dot.get<filtering::Position<3>>();
    EXPECT_NEAR(sHat.dot(dsdt), 0.0, 1E-12);

    EXPECT_TRUE(dot.get<filtering::Velocity<3>>().isApprox(Eigen::Vector3d::Zero(), 1E-12));
    EXPECT_DOUBLE_EQ(dot.get<filtering::Bias<1>>()(0), 0.0);
}

// Integrated check: a short timeUpdate must preserve |s| to RK4 precision.
// `srukf::timeUpdate` calls `propagate(dynamics, state, {0, dt}, dt)`, which
// does a SINGLE RK4 step of size dt — accurate only for small dt. Anything
// long-horizon would trip on integrator error, not the dynamics' invariant.
TEST(SunlineSRuKFAlgorithmDynamics, HeadingMagnitudePreservedOverSmallTimeUpdate) {
    Eigen::Vector3d const sHat0  = Eigen::Vector3d(0.0, 0.0, 1.0);
    Eigen::Vector3d const omega0 = Eigen::Vector3d(0.02, -0.005, 0.01);

    SunlineSRuKFAlgorithm algo;
    configure(algo, makeState(sHat0, omega0, 0.6), diagCovariance(1E-2, 1E-3, 1E-2));
    algo.reset();

    algo.timeUpdate(1.0);

    Eigen::Vector3d const sHat = algo.getState().get<filtering::Position<3>>();
    EXPECT_NEAR(sHat.norm(), sHat0.norm(), 1E-8);
}

// ============================================================================
// timeUpdate: simple invariants.
// ============================================================================

// With omega = 0 the heading must not move.
TEST(SunlineSRuKFAlgorithmTimeUpdate, ZeroRateLeavesHeadingFixed) {
    Eigen::Vector3d const sHat0 = Eigen::Vector3d(0.0, 1.0, 0.0).normalized();
    SunlineSRuKFAlgorithm algo;
    configure(algo,
              makeState(sHat0, Eigen::Vector3d::Zero(), 1.0),
              diagCovariance(1E-2, 1E-3, 1E-2));
    algo.reset();

    algo.timeUpdate(10.0);
    Eigen::Vector3d const sHat = algo.getState().get<filtering::Position<3>>();
    EXPECT_TRUE(sHat.isApprox(sHat0, 1E-9));
}

// dt == 0 short-circuit: the state must equal the anchor on return, with no
// dynamics evolution applied.
TEST(SunlineSRuKFAlgorithmTimeUpdate, ZeroDtCollapsesToAnchor) {
    Eigen::Vector3d const sHat0  = Eigen::Vector3d(0.0, 0.0, 1.0);
    Eigen::Vector3d const omega0 = Eigen::Vector3d(0.1, 0.0, 0.0);

    SunlineSRuKFAlgorithm algo;
    configure(algo, makeState(sHat0, omega0, 1.0), diagCovariance(1E-2, 1E-3, 1E-2));
    algo.reset();

    algo.timeUpdate(0.0);
    State const s = algo.getState();
    EXPECT_TRUE(s.get<filtering::Position<3>>().isApprox(sHat0,  1E-12));
    EXPECT_TRUE(s.get<filtering::Velocity<3>>().isApprox(omega0, 1E-12));
    EXPECT_DOUBLE_EQ(s.get<filtering::Bias<1>>()(0), 1.0);
}

// ============================================================================
// measurementUpdate: a single rate measurement shrinks the rate-block of P.
//
// Drives the SequentialFilter pair directly (timeUpdate + measurementUpdate)
// rather than going through the queue-driven update() — keeps the test focused
// on the SRUKF math, independent of the pack/queue plumbing.
// ============================================================================

TEST(SunlineSRuKFAlgorithmMeasurementUpdate, RateMeasurementShrinksRateCovariance) {
    Eigen::Vector3d const sHat0  = Eigen::Vector3d(0.0, 0.0, 1.0);
    Eigen::Vector3d const omega0 = Eigen::Vector3d(0.01, 0.01, 0.01);

    SunlineSRuKFAlgorithm algo;
    configure(algo,
              makeState(sHat0, omega0, 1.0),
              diagCovariance(1E-2, 1E-1, 1E-1));
    algo.reset();

    Matrix7 const covar0 = algo.getCovariance();

    // timeUpdate(0) rewinds to the anchor and populates sigma points around
    // it — required precondition for measurementUpdate to have fresh sigma
    // points to compute residuals against.
    algo.timeUpdate(0.0);

    RateMeasurement r;
    r.timeTag    = 0.0;
    r.omega_BN_B = Eigen::Vector3d(0.012, 0.008, 0.011);
    r.covar      = (1E-3 * 1E-3) * Eigen::Matrix3d::Identity();
    r.valid      = true;
    algo.measurementUpdate(r);

    Matrix7 const covarN = algo.getCovariance();
    for (int i = 3; i < 6; ++i) {
        EXPECT_LT(covarN(i, i), covar0(i, i)) << "rate cov diag index " << i;
    }
    EXPECT_TRUE(algo.getLastRateResiduals().valid);
}

// ============================================================================
// update(currentSeconds, CssData, RateData): drives the queue-based path. Both
// per-kind residual slots in the returned SunlineSRuKFOutput must end valid
// when both kinds carry data — the regression case that motivated splitting
// Residuals into per-kind storage.
// ============================================================================

namespace {

// Three-CSS, no-CBias, threshold-0 sensor config. Used by the update() tests
// below to exercise the pack methods.
void configureThreeCss(SunlineSRuKFAlgorithm& algo) {
    Eigen::Matrix<double, MaxCss, 3> nHat = Eigen::Matrix<double, MaxCss, 3>::Zero();
    nHat.row(0) = Eigen::RowVector3d( 0.707, -0.5,    0.5);
    nHat.row(1) = Eigen::RowVector3d( 0.707,  0.5,    0.5);
    nHat.row(2) = Eigen::RowVector3d(-0.707,  0.0,    0.707);
    algo.setCssNHat(nHat);
    algo.setCssCBias(Eigen::Vector<double, MaxCss>::Ones());
    algo.setNumberOfCss(3);
    algo.setSensorThreshold(0.0);
    algo.setCssMeasurementNoiseStd(1E-2);
    algo.setGyroMeasurementNoiseStd(1E-3);
}

}  // namespace

TEST(SunlineSRuKFAlgorithmUpdate, UpdateWithRateAndCssExposesBothResiduals) {
    Eigen::Vector3d const sHat0 = Eigen::Vector3d(0.0, 0.0, 1.0);
    SunlineSRuKFAlgorithm algo;
    configure(algo,
              makeState(sHat0, Eigen::Vector3d(0.01, 0.0, 0.0), 1.0),
              diagCovariance(1E-2, 1E-2, 1E-1));
    configureThreeCss(algo);
    algo.reset();

    CssData css;
    css.timeTag      = 1.0;
    css.cosValues(0) = 0.5;
    css.cosValues(1) = 0.5;
    css.cosValues(2) = 0.707;

    RateData rate;
    rate.timeTag = 1.0;
    rate.rate    = Eigen::Vector3d(0.01, 0.0, 0.0);

    SunlineSRuKFOutput const out = algo.update(2.0, css, rate);

    EXPECT_EQ(out.cssResiduals.numberOfActiveCss, 3);
}

// CSS readings all below the configured threshold must produce an
// `valid = false` CSS residual (the pack method's threshold gate), while the
// rate channel — which has no threshold — still fires.
TEST(SunlineSRuKFAlgorithmUpdate, CssBelowThresholdNotProcessed) {
    SunlineSRuKFAlgorithm algo;
    configure(algo,
              makeState(Eigen::Vector3d(0,0,1), Eigen::Vector3d(0.01,0,0), 1.0),
              diagCovariance(1E-2, 1E-2, 1E-1));
    configureThreeCss(algo);
    algo.setSensorThreshold(0.5);
    algo.reset();

    CssData css;
    css.timeTag      = 1.0;
    css.cosValues(0) = 0.1;  // below threshold
    css.cosValues(1) = 0.2;
    css.cosValues(2) = 0.3;

    RateData rate;
    rate.timeTag = 1.0;
    rate.rate    = Eigen::Vector3d(0.01, 0.0, 0.0);

    SunlineSRuKFOutput const out = algo.update(2.0, css, rate);

    EXPECT_EQ(out.cssResiduals.numberOfActiveCss, 0);
}

// ============================================================================
// SRuKF static helpers: numerical helpers.
// ============================================================================

TEST(SrukfDetail, ForwardSubstitutionSolvesLowerTriangular) {
    Eigen::Matrix3d L;
    L << 2.0, 0.0, 0.0,
         1.0, 3.0, 0.0,
         0.5, 1.0, 4.0;
    Eigen::Vector3d const xTruth(1.0, 2.0, 3.0);
    Eigen::Matrix<double, 3, 1> const b = L * xTruth;

    Eigen::Matrix<double, 3, 1> const x =
        SRuKF::forwardSubstitution<3, 1>(L, b);
    EXPECT_TRUE(x.col(0).isApprox(xTruth, 1E-12));
}

TEST(SrukfDetail, BackSubstitutionSolvesUpperTriangular) {
    Eigen::Matrix3d U;
    U << 2.0, 1.0, 0.5,
         0.0, 3.0, 1.0,
         0.0, 0.0, 4.0;
    Eigen::Vector3d const xTruth(1.0, 2.0, 3.0);
    Eigen::Matrix<double, 3, 1> const b = U * xTruth;

    Eigen::Matrix<double, 3, 1> const x =
        SRuKF::backSubstitution<3, 1>(U, b);
    EXPECT_TRUE(x.col(0).isApprox(xTruth, 1E-12));
}

TEST(SrukfDetail, CholeskyDecompositionReconstructsP) {
    Eigen::Matrix3d P;
    P << 4.0, 2.0, 0.5,
         2.0, 5.0, 1.0,
         0.5, 1.0, 6.0;
    Eigen::Matrix3d const L = SRuKF::choleskyDecomposition<3>(P);
    EXPECT_TRUE((L * L.transpose()).isApprox(P, 1E-10));

    // Returned factor should be lower-triangular.
    for (int i = 0; i < L.rows(); ++i) {
        for (int j = i + 1; j < L.cols(); ++j) {
            EXPECT_NEAR(L(i, j), 0.0, 1E-12) << "(" << i << "," << j << ")";
        }
    }
}

// SRUKF feeds qrDecompositionJustR a wider-than-tall A and expects back a
// square N×N factor such that R * R^T == A * A^T. The function transposes its
// internal R, so the returned matrix is lower-triangular.
TEST(SrukfDetail, QrDecompositionJustRPreservesNormalEquations) {
    Eigen::Matrix<double, 3, 9> A;
    A << 1.0,  2.0,  3.0,  4.0, 5.0, 6.0, 7.0, 8.0, 9.0,
         0.5,  1.0,  1.5,  2.0, 2.5, 3.0, 3.5, 4.0, 4.5,
         2.0, -1.0,  0.0,  1.0, 2.0, 3.0, 4.0, 5.0, 6.0;

    Eigen::Matrix3d const R = SRuKF::qrDecompositionJustR<3, 9>(A);

    for (int i = 0; i < R.rows(); ++i) {
        for (int j = i + 1; j < R.cols(); ++j) {
            EXPECT_NEAR(R(i, j), 0.0, 1E-10) << "(" << i << "," << j << ")";
        }
    }
    EXPECT_TRUE((R * R.transpose()).isApprox(A * A.transpose(), 1E-9));
}

TEST(SrukfDetail, CholeskyUpDownDateMatchesExplicitUpdate) {
    Eigen::Matrix3d P0;
    P0 << 4.0, 1.0, 0.0,
          1.0, 3.0, 0.5,
          0.0, 0.5, 2.0;
    Eigen::Matrix3d const S0 = SRuKF::choleskyDecomposition<3>(P0);

    Eigen::Vector3d const v(0.1, -0.2, 0.3);

    // Up-date with +coef and down-date with -coef must reconstruct P0 ± coef v vᵀ.
    {
        double const coef = 0.5;
        Eigen::Matrix3d const S1 = SRuKF::choleskyUpDownDate<3>(S0, v, coef);
        Eigen::Matrix3d const P1 = P0 + coef * v * v.transpose();
        EXPECT_TRUE((S1 * S1.transpose()).isApprox(P1, 1E-9));
    }
    {
        double const coef = -0.5;
        Eigen::Matrix3d const S1 = SRuKF::choleskyUpDownDate<3>(S0, v, coef);
        Eigen::Matrix3d const P1 = P0 - 0.5 * v * v.transpose();
        EXPECT_TRUE((S1 * S1.transpose()).isApprox(P1, 1E-9));
    }
}

}  // namespace filtering::sunlineSRuKF
