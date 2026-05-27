// SPDX-License-Identifier: ISC
// Copyright (c) 2026, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

// Direct unit tests for FlybyODuKFAlgorithm — no xmera, no messaging. Mirrors
// the intent of the Python integration test
// (fswAlgorithms/opticalNavigation/flybyODuKF/_UnitTest/test_flybyODuKF.py):
// two-body propagation conserves energy and grows the covariance without
// measurements, and heading measurements shrink the covariance and pull the
// estimate's heading onto the truth.

#include "flybyODuKFAlgorithm.h"

#include <gtest/gtest.h>

#include <Eigen/Dense>

#include <cmath>
#include <utility>

namespace filtering::flybyODuKF {
namespace {

using State = FlybyODuKFAlgorithm::State;
using Vector6 = Eigen::Matrix<double, 6, 1>;
using Matrix6 = Eigen::Matrix<double, 6, 6>;

// Mars-like central body, matching the Python test's tunables.
constexpr double kMu             = 42828.314 * 1E9;  // [m^3/s^2]
constexpr double kUnitConversion = 1E-3;             // SI (m) -> km
constexpr double kAlpha          = 0.02;
constexpr double kBeta           = 2.0;

// Reference two-body derivative on an SI [m, m/s] 6-state.
Vector6 twoBodyDeriv(Vector6 const& x) {
    Vector6 dx;
    dx.head<3>() = x.tail<3>();
    dx.tail<3>() = -kMu / std::pow(x.head<3>().norm(), 3) * x.head<3>();
    return dx;
}

// One classic RK4 step — the same integrator the filter uses internally, so
// the filter's propagated mean should match this to near machine precision.
Vector6 rk4Step(Vector6 const& x, double dt) {
    Vector6 const k1 = twoBodyDeriv(x);
    Vector6 const k2 = twoBodyDeriv(x + 0.5 * dt * k1);
    Vector6 const k3 = twoBodyDeriv(x + 0.5 * dt * k2);
    Vector6 const k4 = twoBodyDeriv(x + dt * k3);
    return x + (dt / 6.0) * (k1 + 2.0 * k2 + 2.0 * k3 + k4);
}

double orbitalEnergy(Vector6 const& x) {
    return 0.5 * x.tail<3>().squaredNorm() - kMu / x.head<3>().norm();
}

Matrix6 initialCovariance() {
    Vector6 d;
    d << 1000. * 1E6, 1000. * 1E6, 1000. * 1E6, 0.1 * 1E6, 0.1 * 1E6, 0.1 * 1E6;
    return d.asDiagonal();
}

Matrix6 processNoise() {
    double const sigmaPos = (1E-8) * (1E-8);
    double const sigmaVel = (1E-10) * (1E-10);
    Vector6 d;
    d << sigmaPos, sigmaPos, sigmaPos, sigmaVel, sigmaVel, sigmaVel;
    return d.asDiagonal();
}

// Periapsis state of an a=4000 km, e=0.2 orbit: position along +x, velocity
// purely tangential (+y). Any valid two-body state works for these checks.
Vector6 initialTruth() {
    double const a  = 4000. * 1E3;
    double const e  = 0.2;
    double const rp = a * (1.0 - e);
    double const vp = std::sqrt(kMu / a * (1.0 + e) / (1.0 - e));
    Vector6 x;
    x << rp, 0.0, 0.0, 0.0, vp, 0.0;
    return x;
}

State toState(Vector6 const& x) {
    State s;
    s.set<filtering::Position<3>>(x.head<3>());
    s.set<filtering::Velocity<3>>(x.tail<3>());
    return s;
}

Vector6 stateVector(FilterStateOutput const& out) { return out.state; }

void configure(FlybyODuKFAlgorithm& algo, Vector6 const& initial) {
    algo.setAlpha(kAlpha);
    algo.setBeta(kBeta);
    algo.setUnitConversion(kUnitConversion);
    algo.setMu(kMu);
    algo.setInitialState(toState(initial));
    algo.setInitialCovariance(initialCovariance());
    algo.setProcessNoise(processNoise());
}

}  // namespace

TEST(FlybyODuKFAlgorithm, ConfigRoundTrip) {
    FlybyODuKFAlgorithm algo;
    algo.setMu(kMu);
    algo.setAlpha(kAlpha);
    algo.setBeta(kBeta);
    algo.setUnitConversion(kUnitConversion);
    algo.setMeasurementNoiseScale(3.5);
    algo.setProcessNoise(processNoise());
    algo.setInitialCovariance(initialCovariance());
    algo.setInitialState(toState(initialTruth()));

    EXPECT_DOUBLE_EQ(algo.getMu(), kMu);
    EXPECT_DOUBLE_EQ(algo.getAlpha(), kAlpha);
    EXPECT_DOUBLE_EQ(algo.getBeta(), kBeta);
    EXPECT_DOUBLE_EQ(algo.getUnitConversion(), kUnitConversion);
    EXPECT_DOUBLE_EQ(algo.getMeasurementNoiseScale(), 3.5);
    EXPECT_TRUE(algo.getProcessNoise().isApprox(processNoise()));
    EXPECT_TRUE(algo.getInitialCovariance().isApprox(initialCovariance()));
    EXPECT_TRUE(algo.getInitialState().raw().isApprox(initialTruth()));
}

TEST(FlybyODuKFAlgorithm, PropagationConservesEnergyAndGrowsCovariance) {
    Vector6 const x0 = initialTruth();
    FlybyODuKFAlgorithm algo;
    configure(algo, x0);
    algo.reset();

    Matrix6 const sqrtCovar0 = algo.getState().sqrtCovar;
    double const covarNorm0  = (sqrtCovar0 * sqrtCovar0.transpose()).norm();
    double const energy0     = orbitalEnergy(x0);

    constexpr double dt    = 10.0;
    constexpr int    steps = 360;  // 60 minutes, as in the Python test
    Vector6 truth = x0;
    for (int i = 0; i < steps; ++i) {
        truth = rk4Step(truth, dt);
        algo.timeUpdate(dt);

        Vector6 const estimate = stateVector(algo.getState());
        // The filter's mean is a deterministic RK4 propagation in the
        // converted unit system; recovered to SI it must track the reference.
        EXPECT_TRUE(estimate.isApprox(truth, 1E-6)) << "step " << i;
        EXPECT_NEAR(orbitalEnergy(estimate), energy0, std::abs(energy0) * 1E-2) << "step " << i;
    }

    Matrix6 const sqrtCovarN = algo.getState().sqrtCovar;
    double const covarNormN  = (sqrtCovarN * sqrtCovarN.transpose()).norm();
    // Without measurements the uncertainty must grow substantially (Python
    // requires > 5x over the same span).
    EXPECT_GT(covarNormN, 5.0 * covarNorm0);
}

TEST(FlybyODuKFAlgorithm, MeasurementUpdatesShrinkCovarianceAndAlignHeading) {
    Vector6 const x0 = initialTruth();

    // Start the estimate offset from truth so the heading measurements have
    // something to correct.
    Vector6 x0Estimate = x0;
    x0Estimate(0) += 5.0 * 1E3;  // +5 km cross-range bends the heading
    x0Estimate(4) += 5.0;        // +5 m/s

    FlybyODuKFAlgorithm algo;
    configure(algo, x0Estimate);
    algo.reset();

    Matrix6 const sqrtCovar0 = algo.getState().sqrtCovar;
    Matrix6 const covar0     = sqrtCovar0 * sqrtCovar0.transpose();

    constexpr double dt    = 1.0;
    constexpr int    steps = 250;
    Vector6 truth = x0;
    for (int i = 1; i <= steps; ++i) {
        truth = rk4Step(truth, dt);
        algo.timeUpdate(dt);

        if (i % 10 == 0) {
            HeadingMeasurement meas;
            meas.timeTag   = static_cast<double>(i) * dt;
            meas.rhat_BN_N = truth.head<3>().normalized();
            meas.covarN    = 5E-5 * Eigen::Matrix3d::Identity();
            meas.valid     = true;
            algo.measurementUpdate(meas);
        }
    }

    Matrix6 const sqrtCovarN = algo.getState().sqrtCovar;
    Matrix6 const covarN     = sqrtCovarN * sqrtCovarN.transpose();

    // Velocity-block uncertainty must shrink relative to the initial guess.
    for (int i = 3; i < 6; ++i) {
        EXPECT_LT(covarN(i, i), covar0(i, i)) << "velocity covariance index " << i;
    }

    // Heading is directly observed, so the estimate's position direction must
    // converge onto the truth direction.
    Vector6 const estimate = stateVector(algo.getState());
    Eigen::Vector3d const estHeading   = estimate.head<3>().normalized();
    Eigen::Vector3d const truthHeading = truth.head<3>().normalized();
    EXPECT_GT(estHeading.dot(truthHeading), std::cos(1.0 * M_PI / 180.0));

    // Residuals were captured on the last update.
    EXPECT_TRUE(algo.getLastResiduals().valid);
}

// The host adapter drives the filter through the measurement_queue: enqueue a
// measurement, then a single update(t0, t1) per window. This must produce the
// same convergence/covariance behavior as driving timeUpdate/measurementUpdate
// directly.
TEST(FlybyODuKFAlgorithm, QueueDrivenUpdateConvergesLikeDirectPath) {
    Vector6 const x0 = initialTruth();

    Vector6 x0Estimate = x0;
    x0Estimate(0) += 5.0 * 1E3;
    x0Estimate(4) += 5.0;

    FlybyODuKFAlgorithm algo;
    configure(algo, x0Estimate);
    algo.reset();

    Matrix6 const covar0 = [&] {
        Matrix6 const s = algo.getState().sqrtCovar;
        return Matrix6(s * s.transpose());
    }();

    constexpr double dt    = 1.0;
    constexpr int    steps = 250;
    Vector6 truth = x0;
    for (int i = 1; i <= steps; ++i) {
        double const t0 = static_cast<double>(i - 1) * dt;
        double const t1 = static_cast<double>(i) * dt;
        truth = rk4Step(truth, dt);

        if (i % 10 == 0) {
            HeadingMeasurement meas;
            meas.timeTag   = t1;
            meas.rhat_BN_N = truth.head<3>().normalized();
            meas.covarN    = 5E-5 * Eigen::Matrix3d::Identity();
            meas.valid     = true;
            algo.enqueueMeasurement(meas.timeTag, std::move(meas));
        }

        // Single drive entry point over the window; the queue decides whether
        // each step is a time update or a measurement update.
        algo.update(t0, t1);
    }

    Matrix6 const sqrtCovarN = algo.getState().sqrtCovar;
    Matrix6 const covarN     = sqrtCovarN * sqrtCovarN.transpose();
    for (int i = 3; i < 6; ++i) {
        EXPECT_LT(covarN(i, i), covar0(i, i)) << "velocity covariance index " << i;
    }

    Vector6 const estimate = stateVector(algo.getState());
    Eigen::Vector3d const estHeading   = estimate.head<3>().normalized();
    Eigen::Vector3d const truthHeading = truth.head<3>().normalized();
    EXPECT_GT(estHeading.dot(truthHeading), std::cos(1.0 * M_PI / 180.0));
}

}  // namespace filtering::flybyODuKF
