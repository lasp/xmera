// SPDX-License-Identifier: ISC
// Copyright (c) 2026, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

#include "encoderTestHelpers.hpp"
#include <architecture/utilities/simDefinitions.h>

#include <gtest/gtest.h>

#include <numbers>

using encodertest::EncoderHarness;
using encodertest::oneSecond;

namespace {
    constexpr double pi = std::numbers::pi;
    constexpr double tolerance = 1e-9;
}  // namespace

//! On the first step the time step is zero, so the encoder sends the true wheel speeds.
TEST(Encoder, firstStepSendsTrueSpeeds) {
    EncoderHarness harness(3, 2);
    harness.encoder.reset(0);

    RWSpeedMsgPayload const out = harness.step({100.0, 200.0, 300.0}, 0);

    EXPECT_DOUBLE_EQ(out.wheelSpeeds[0], 100.0);
    EXPECT_DOUBLE_EQ(out.wheelSpeeds[1], 200.0);
    EXPECT_DOUBLE_EQ(out.wheelSpeeds[2], 300.0);
}

//! With two clicks per rotation, each output speed is a whole number of clicks times pi rad/s.
//! The encoder keeps the fraction of a click that remains and adds it to the next step.
TEST(Encoder, quantizesSpeedOverConsecutiveSteps) {
    EncoderHarness harness(3, 2);
    harness.encoder.reset(0);
    std::vector<double> const speeds{100.0, 200.0, 300.0};

    harness.step(speeds, 0);
    RWSpeedMsgPayload const first = harness.step(speeds, oneSecond);
    RWSpeedMsgPayload const second = harness.step(speeds, 2 * oneSecond);

    EXPECT_NEAR(first.wheelSpeeds[0], 31.0 * pi, tolerance);
    EXPECT_NEAR(first.wheelSpeeds[1], 63.0 * pi, tolerance);
    EXPECT_NEAR(first.wheelSpeeds[2], 95.0 * pi, tolerance);
    EXPECT_NEAR(second.wheelSpeeds[0], 32.0 * pi, tolerance);
    EXPECT_NEAR(second.wheelSpeeds[1], 64.0 * pi, tolerance);
    EXPECT_NEAR(second.wheelSpeeds[2], 95.0 * pi, tolerance);
}

//! When the signal is off, the encoder sends zero speed.
TEST(Encoder, offSignalSendsZeroSpeed) {
    EncoderHarness harness(3, 2);
    harness.encoder.reset(0);
    std::vector<double> const speeds{100.0, 200.0, 300.0};
    harness.step(speeds, 0);
    harness.step(speeds, oneSecond);

    for (int i = 0; i < 3; ++i) { harness.encoder.rwSignalState[i] = SIGNAL_OFF; }
    RWSpeedMsgPayload const out = harness.step(speeds, 2 * oneSecond);

    EXPECT_DOUBLE_EQ(out.wheelSpeeds[0], 0.0);
    EXPECT_DOUBLE_EQ(out.wheelSpeeds[1], 0.0);
    EXPECT_DOUBLE_EQ(out.wheelSpeeds[2], 0.0);
}
