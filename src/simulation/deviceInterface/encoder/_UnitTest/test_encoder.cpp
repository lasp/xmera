// SPDX-License-Identifier: ISC
// Copyright (c) 2026, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

#include "encoderTestHelpers.hpp"

#include <gtest/gtest.h>

using encodertest::EncoderHarness;

//! On the first step the time step is zero, so the encoder sends the true wheel speeds.
TEST(Encoder, firstStepSendsTrueSpeeds) {
    EncoderHarness harness(3, 2);
    harness.encoder.reset(0);

    RWSpeedMsgPayload const out = harness.step({100.0, 200.0, 300.0}, 0);

    EXPECT_DOUBLE_EQ(out.wheelSpeeds[0], 100.0);
    EXPECT_DOUBLE_EQ(out.wheelSpeeds[1], 200.0);
    EXPECT_DOUBLE_EQ(out.wheelSpeeds[2], 300.0);
}
