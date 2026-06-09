// SPDX-License-Identifier: ISC
// Copyright (c) 2026, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

// Unit tests for FlybyODuKFAlgorithm — the single composition root that both
// satisfies SequentialFilter<…, HeadingMeasurement> (via public timeUpdate /
// measurementUpdate) and drives itself through an internal measurement_queue
// via update(t0, t1). No xmera, no messaging.
//
// Mirrors the intent of the Python integration test
// (fswAlgorithms/opticalNavigation/flybyODuKF/_UnitTest/test_flybyODuKF.py):
// two-body propagation conserves energy and grows the covariance without
// measurements, and heading measurements shrink the velocity covariance and
// pull the estimate's heading onto the truth. The QueueDriven case proves
// the queue path produces the same convergence as direct stepping.

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

Vector6 twoBodyDeriv(Vector6 const& x) {
    Vector6 dx;
    dx.head<3>() = x.tail<3>();
    dx.tail<3>() = -kMu / std::pow(x.head<3>().norm(), 3) * x.head<3>();
    return dx;
}

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

// FilterStateOutput is already in SI (the algorithm undoes the internal
// unitConversion in getState()).
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

// Drive the algorithm step-by-step via the SequentialFilter pair
// (timeUpdate). No measurements — orbital energy is conserved and the
// uncertainty grows.
TEST(FlybyODuKFAlgorithm, PropagationConservesEnergyAndGrowsCovariance) {
    Vector6 const x0 = initialTruth();
    FlybyODuKFAlgorithm algo;
    configure(algo, x0);
    algo.reset();

    double const covarNorm0  = algo.getCovariance().norm();
    double const energy0     = orbitalEnergy(x0);

    // timeUpdate(T) rewinds to the last-measurement state and propagates by T,
    // so each iteration here re-integrates from x0 to t = i*dt. Cost is
    // O(steps²); keep steps modest.
    constexpr double dt    = 10.0;
    constexpr int    steps = 60;  // 10-minute window
    Vector6 truth = x0;
    for (int i = 1; i <= steps; ++i) {
        truth = rk4Step(truth, dt);
        algo.timeUpdate(static_cast<double>(i) * dt);

        Vector6 const estimate = stateVector(algo.getState());
        EXPECT_TRUE(estimate.isApprox(truth, 1E-6)) << "step " << i;
        EXPECT_NEAR(orbitalEnergy(estimate), energy0, std::abs(energy0) * 1E-2) << "step " << i;
    }

    double const covarNormN  = algo.getCovariance().norm();
    EXPECT_GT(covarNormN, 5.0 * covarNorm0);
}

// Drive the algorithm step-by-step with heading measurements folded in
// directly via measurementUpdate. The velocity-block covariance shrinks and
// the estimate's position heading converges onto truth.
TEST(FlybyODuKFAlgorithm, MeasurementUpdatesShrinkCovarianceAndAlignHeading) {
    Vector6 const x0 = initialTruth();

    Vector6 x0Estimate = x0;
    x0Estimate(0) += 5.0 * 1E3;  // +5 km cross-range bends the heading
    x0Estimate(4) += 5.0;        // +5 m/s

    FlybyODuKFAlgorithm algo;
    configure(algo, x0Estimate);
    algo.reset();

    Matrix6 const covar0 = algo.getCovariance();

    // timeUpdate(dt) is dt since the last measurement — track it manually.
    constexpr double dt    = 1.0;
    constexpr int    steps = 250;
    Vector6 truth = x0;
    double  lastMeasTime = 0.0;
    for (int i = 1; i <= steps; ++i) {
        truth = rk4Step(truth, dt);
        double const currentTime = static_cast<double>(i) * dt;
        algo.timeUpdate(currentTime - lastMeasTime);

        if (i % 10 == 0) {
            HeadingMeasurement meas;
            meas.timeTag   = currentTime;
            meas.rhat_BN_N = truth.head<3>().normalized();
            meas.covarN    = 5E-5 * Eigen::Matrix3d::Identity();
            meas.valid     = true;
            algo.measurementUpdate(meas);
            lastMeasTime = currentTime;
        }
    }

    Matrix6 const covarN = algo.getCovariance();
    for (int i = 3; i < 6; ++i) {
        EXPECT_LT(covarN(i, i), covar0(i, i)) << "velocity covariance index " << i;
    }

    Vector6 const estimate = stateVector(algo.getState());
    Eigen::Vector3d const estHeading   = estimate.head<3>().normalized();
    Eigen::Vector3d const truthHeading = truth.head<3>().normalized();
    EXPECT_GT(estHeading.dot(truthHeading), std::cos(1.0 * M_PI / 180.0));

    EXPECT_TRUE(algo.getLastResiduals().valid);
}

// Same scenario as MeasurementUpdates… but driven through the queue: enqueue
// at the measurement time, then a single update(t0, t1) per step. Proves the
// queue-driven path (apply_sequential against *this) produces the same
// behavior as direct stepping.
TEST(FlybyODuKFAlgorithm, QueueDrivenUpdateConvergesLikeDirectPath) {
    Vector6 const x0 = initialTruth();

    Vector6 x0Estimate = x0;
    x0Estimate(0) += 5.0 * 1E3;
    x0Estimate(4) += 5.0;

    FlybyODuKFAlgorithm algo;
    configure(algo, x0Estimate);
    algo.reset();

    Matrix6 const covar0 = algo.getState().covariance;

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

        algo.update(t0, t1);
    }

    Matrix6 const covarN = algo.getState().covariance;
    for (int i = 3; i < 6; ++i) {
        EXPECT_LT(covarN(i, i), covar0(i, i)) << "velocity covariance index " << i;
    }

    Vector6 const estimate = stateVector(algo.getState());
    Eigen::Vector3d const estHeading   = estimate.head<3>().normalized();
    Eigen::Vector3d const truthHeading = truth.head<3>().normalized();
    EXPECT_GT(estHeading.dot(truthHeading), std::cos(1.0 * M_PI / 180.0));
}

}  // namespace filtering::flybyODuKF
