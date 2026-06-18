// SPDX-License-Identifier: ISC
// Copyright (c) 2018, Autonomous Vehicle System Lab, University of Colorado at Boulder
// Copyright (c) 2023, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

#include "architecture/utilities/gauss_markov.h"

#include <gtest/gtest.h>

#include <cmath>
#include <Eigen/Dense>

Eigen::Vector2d calculateSD(Eigen::MatrixXd dat, int64_t numPts) {
    Eigen::Vector2d sum = dat.rowwise().sum();

    Eigen::Vector2d means = sum / numPts;
    Eigen::MatrixXd mean;
    mean.resize(2, numPts);
    mean.block(0, 0, 1, numPts).fill(means(0) / numPts);
    mean.block(1, 0, 1, numPts).fill(means(1) / numPts);

    Eigen::MatrixXd resid = dat - mean;
    resid = resid.cwiseProduct(resid);
    Eigen::MatrixXd stnd = resid.rowwise().sum();
    stnd = stnd / numPts;
    stnd = stnd.array().sqrt().matrix();

    return stnd;
}

TEST(GaussMarkov, stdDeviationIsExpected) {
    // Test if the std deviation is what we asked for
    uint64_t seedIn = 1'000;
    Eigen::Matrix2d propIn;
    propIn << 1, 1, 0, 1;
    Eigen::Matrix2d covar;
    covar << 1'500, 0, 0, 1.5;
    Eigen::Vector2d bounds;
    bounds << 1e-15, 1e-15;  // small but non-zero required for "white" noise
    GaussMarkov<2> errorModel;
    errorModel.setRNGSeed(seedIn);
    errorModel.setPropMatrix(propIn);
    errorModel.setNoiseMatrix(covar);
    errorModel.setUpperBounds(bounds);
    int64_t numPts = 100'000;

    Eigen::MatrixXd noiseOut;
    noiseOut.resize(2, numPts);

    for (int64_t i = 0; i < numPts; i++) {
        errorModel.computeNextState();
        noiseOut.block(0, i, 2, 1) = errorModel.getCurrentState();
    }

    // Test if the std deviation is what we asked for
    Eigen::Vector2d stds = calculateSD(noiseOut, numPts);
    Eigen::Vector2d stdsIn;
    stdsIn(0) = covar(0, 0) / 1.5;
    stdsIn(1) = covar(1, 1) / 1.5;
    EXPECT_LT((stdsIn(0) - stds(0)) / (stdsIn(0)), 0.1);
    EXPECT_LT((stdsIn(1) - stds(1)) / (stdsIn(1)), 0.1);
}

TEST(GaussMarkov, meanIsZero) {
    // Test if the mean is zero
    uint64_t seedIn = 1'000;
    Eigen::Matrix2d propIn;
    propIn << 1, 1, 0, 1;
    Eigen::Matrix2d covar;
    covar << 1'500, 0, 0, 1.5;
    Eigen::Vector2d bounds;
    bounds << 1e-15, 1e-15;  // small but non-zero required for "white" noise
    GaussMarkov<2> errorModel;
    errorModel.setRNGSeed(seedIn);
    errorModel.setPropMatrix(propIn);
    errorModel.setNoiseMatrix(covar);
    errorModel.setUpperBounds(bounds);
    int64_t numPts = 100'000;

    Eigen::MatrixXd noiseOut;
    noiseOut.resize(2, numPts);

    for (int64_t i = 0; i < numPts; i++) {
        errorModel.computeNextState();
        noiseOut.block(0, i, 2, 1) = errorModel.getCurrentState();
    }

    Eigen::Vector2d means = noiseOut.rowwise().mean();
    Eigen::Vector2d meansIn;
    meansIn << 0, 0;
    EXPECT_LT(fabs(meansIn(0) - means(0)), 5);
    EXPECT_LT(fabs(meansIn(1) - means(1)), 0.05);
}

TEST(GaussMarkov, boundsAreRespected) {
    // Test if the bounds are obeyed
    uint64_t seedIn = 1'500;
    Eigen::Matrix2d propIn;
    propIn << 1, 0, 0, 1;
    Eigen::Matrix2d covar;
    covar << 1.5, 0, 0, 0.015;
    Eigen::Vector2d bounds;
    bounds << 10., 0.1;
    GaussMarkov<2> errorModel;
    errorModel.setRNGSeed(seedIn);
    errorModel.setPropMatrix(propIn);
    errorModel.setNoiseMatrix(covar);
    errorModel.setUpperBounds(bounds);

    int64_t numPts = 100'000;
    Eigen::MatrixXd noiseOut;
    noiseOut.resize(2, numPts);

    Eigen::Vector2d maxOut;
    maxOut.fill(0.0);
    Eigen::Vector2d minOut;
    minOut.fill(0.0);

    numPts = (int64_t) 1e6;
    noiseOut.resize(2, numPts);

    for (int64_t i = 0; i < numPts; i++) {
        errorModel.computeNextState();
        noiseOut.block(0, i, 2, 1) = errorModel.getCurrentState();
        if (noiseOut(0, i) > maxOut(0)) { maxOut(0) = noiseOut(0, i); }
        if (noiseOut(0, i) < minOut(0)) { minOut(0) = noiseOut(0, i); }
        if (noiseOut(1, i) > maxOut(1)) { maxOut(1) = noiseOut(1, i); }
        if (noiseOut(1, i) < minOut(1)) { minOut(1) = noiseOut(1, i); }
    }

    EXPECT_LT(fabs(12.481655180914322 - maxOut(0)) / 12.481655180914322, 5e-1);
    EXPECT_LT(fabs(0.12052269089286843 - maxOut(1)) / 0.12052269089286843, 5e-1);
    EXPECT_LT(fabs(-12.230618182796439 - minOut(0)) / -12.230618182796439, 5e-1);
    EXPECT_LT(fabs(-0.12055787311661936 - minOut(1)) / -0.12055787311661936, 5e-1);
}

TEST(GaussMarkov, resetAndSetCurrentState) {
    GaussMarkov<3> model(42);
    model.setNoiseMatrix(Eigen::Matrix3d::Identity());
    model.setPropMatrix(Eigen::Matrix3d::Identity());
    model.setUpperBounds(Eigen::Vector3d::Constant(-1.0));  // non-positive disables clamping

    for (int i = 0; i < 100; i++) { model.computeNextState(); }
    EXPECT_GT(model.getCurrentState().norm(), 0.0);  // the walk has moved away from zero

    model.reset();
    EXPECT_EQ(model.getCurrentState().norm(), 0.0);  // reset returns to zero

    Eigen::Vector3d target(1.0, 2.0, 3.0);
    model.setCurrentState(target);
    EXPECT_EQ((model.getCurrentState() - target).norm(), 0.0);
}

TEST(GaussMarkov, deterministicForSameSeed) {
    // Two identically-seeded, identically-configured models must produce the same sequence.
    GaussMarkov<3> a(777);
    GaussMarkov<3> b(777);
    for (auto* m : {&a, &b}) {
        m->setNoiseMatrix(Eigen::Matrix3d::Identity());
        m->setPropMatrix(Eigen::Matrix3d::Identity() * 0.9);
        m->setUpperBounds(Eigen::Vector3d::Constant(-1.0));
    }
    for (int i = 0; i < 50; i++) {
        a.computeNextState();
        b.computeNextState();
    }
    EXPECT_EQ((a.getCurrentState() - b.getCurrentState()).norm(), 0.0);
}

template<int N>
static void runFiniteSteps() {
    GaussMarkov<N> m(123);
    m.setNoiseMatrix(Eigen::Matrix<double, N, N>::Identity());
    m.setPropMatrix(Eigen::Matrix<double, N, N>::Identity());
    m.setUpperBounds(Eigen::Matrix<double, N, 1>::Constant(5.0));
    for (int i = 0; i < 100; i++) { m.computeNextState(); }
    EXPECT_TRUE(m.getCurrentState().allFinite());
}

TEST(GaussMarkov, runsForVariousSizes) {
    runFiniteSteps<1>();
    runFiniteSteps<3>();
    runFiniteSteps<12>();
    runFiniteSteps<18>();
}

TEST(GaussMarkov, memorylessWhenPropZero) {
    // propMatrix = 0 makes each step independent white noise: std = sigma/3 and lag-1 autocorr ~ 0.
    GaussMarkov<1> m(2'024);
    Eigen::Matrix<double, 1, 1> sigma;
    sigma(0, 0) = 30.0;
    m.setNoiseMatrix(sigma);
    m.setPropMatrix(Eigen::Matrix<double, 1, 1>::Zero());
    Eigen::Matrix<double, 1, 1> noBound;
    noBound(0, 0) = -1.0;  // disable clamping
    m.setUpperBounds(noBound);

    int64_t numPts = 200'000;
    double sum = 0.0, sumSq = 0.0, lagSum = 0.0, prev = 0.0;
    for (int64_t i = 0; i < numPts; i++) {
        m.computeNextState();
        double x = m.getCurrentState()(0, 0);
        sum += x;
        sumSq += x * x;
        if (i > 0) { lagSum += x * prev; }
        prev = x;
    }
    double mean = sum / numPts;
    double var = sumSq / numPts - mean * mean;
    double stdDev = std::sqrt(var);
    double expectedStd = 30.0 / 3.0;  // sigma / 3
    EXPECT_LT(fabs(stdDev - expectedStd) / expectedStd, 0.05);

    double lag1Corr = (lagSum / (numPts - 1) - mean * mean) / var;
    EXPECT_LT(fabs(lag1Corr), 0.02);  // memoryless: negligible autocorrelation
}
