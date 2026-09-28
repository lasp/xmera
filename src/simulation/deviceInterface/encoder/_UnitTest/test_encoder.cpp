// SPDX-License-Identifier: ISC
// Copyright (c) 2026, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

#include "encoderTestHelpers.hpp"

#include <gtest/gtest.h>

#include <numbers>
#include <stdexcept>

using encodertest::EncoderHarness;
using encodertest::oneSecond;

namespace {
    constexpr double pi = std::numbers::pi;
    constexpr double tolerance = 1e-9;
}  // namespace

//! On the first step, the time step is zero. Thus the encoder sends the input wheel speeds.
TEST(Encoder, firstStepSendsTrueSpeeds) {
    EncoderHarness harness(3, 2);
    harness.encoder.reset(0);

    RWSpeedMsgPayload const out = harness.step({100.0, 200.0, 300.0}, 0);

    EXPECT_DOUBLE_EQ(out.wheelSpeeds[0], 100.0);
    EXPECT_DOUBLE_EQ(out.wheelSpeeds[1], 200.0);
    EXPECT_DOUBLE_EQ(out.wheelSpeeds[2], 300.0);
}

//! With two clicks per rotation, each output speed is pi rad/s multiplied by an integer number of clicks.
//! The encoder keeps the remaining part of a click and adds it to the next step.
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

    for (std::size_t i = 0; i < 3; ++i) { harness.encoder.setSignalState(i, EncoderSignal::Off); }
    RWSpeedMsgPayload const out = harness.step(speeds, 2 * oneSecond);

    EXPECT_DOUBLE_EQ(out.wheelSpeeds[0], 0.0);
    EXPECT_DOUBLE_EQ(out.wheelSpeeds[1], 0.0);
    EXPECT_DOUBLE_EQ(out.wheelSpeeds[2], 0.0);
}

//! An off signal erases the remaining clicks. After the signal is nominal again, the count starts from zero.
TEST(Encoder, nominalSignalAfterOffStartsFromZeroClicks) {
    EncoderHarness harness(3, 2);
    harness.encoder.reset(0);
    std::vector<double> const speeds{100.0, 200.0, 300.0};
    harness.step(speeds, 0);
    harness.step(speeds, oneSecond);
    harness.step(speeds, 2 * oneSecond);

    for (std::size_t i = 0; i < 3; ++i) { harness.encoder.setSignalState(i, EncoderSignal::Off); }
    harness.step(speeds, 3 * oneSecond);
    for (std::size_t i = 0; i < 3; ++i) { harness.encoder.setSignalState(i, EncoderSignal::Nominal); }
    RWSpeedMsgPayload const out = harness.step({500.0, 400.0, 300.0}, 4 * oneSecond);

    EXPECT_NEAR(out.wheelSpeeds[0], 159.0 * pi, tolerance);
    EXPECT_NEAR(out.wheelSpeeds[1], 127.0 * pi, tolerance);
    EXPECT_NEAR(out.wheelSpeeds[2], 95.0 * pi, tolerance);
}

//! When the signal is stuck, the encoder sends the speeds of the previous step and ignores the new input.
TEST(Encoder, stuckSignalHoldsPreviousSpeed) {
    EncoderHarness harness(3, 2);
    harness.encoder.reset(0);
    harness.step({500.0, 400.0, 300.0}, 0);
    RWSpeedMsgPayload const before = harness.step({500.0, 400.0, 300.0}, oneSecond);

    for (std::size_t i = 0; i < 3; ++i) { harness.encoder.setSignalState(i, EncoderSignal::Stuck); }
    RWSpeedMsgPayload const out = harness.step({100.0, 200.0, 300.0}, 2 * oneSecond);

    EXPECT_DOUBLE_EQ(out.wheelSpeeds[0], before.wheelSpeeds[0]);
    EXPECT_DOUBLE_EQ(out.wheelSpeeds[1], before.wheelSpeeds[1]);
    EXPECT_DOUBLE_EQ(out.wheelSpeeds[2], before.wheelSpeeds[2]);
}

//! The encoder cannot operate without an input message. Reset rejects an encoder with no connected input message.
TEST(Encoder, resetRejectsUnlinkedInputMessage) {
    Encoder encoder(3, 2);

    EXPECT_THROW(encoder.reset(0), std::invalid_argument);
}

//! The setters reject zero for the wheel count and for the clicks per rotation.
TEST(Encoder, settersRejectZero) {
    Encoder encoder(3, 2);

    EXPECT_THROW(encoder.setNumRW(0), std::invalid_argument);
    EXPECT_THROW(encoder.setClicksPerRotation(0), std::invalid_argument);
}

//! The output message has RW_EFF_CNT wheel slots. The wheel count setter rejects a count that is more than RW_EFF_CNT.
TEST(Encoder, setNumRWRejectsCountAboveRwEffCnt) {
    Encoder encoder(3, 2);

    EXPECT_NO_THROW(encoder.setNumRW(RW_EFF_CNT));
    EXPECT_THROW(encoder.setNumRW(RW_EFF_CNT + 1), std::invalid_argument);
    EXPECT_EQ(encoder.getNumRW(), static_cast<std::size_t>(RW_EFF_CNT));
}

//! The signal state setter rejects a wheel index that is not less than the wheel count, and a state that is not an
//! EncoderSignal value.
TEST(Encoder, setSignalStateRejectsInvalidInput) {
    Encoder encoder(3, 2);

    EXPECT_THROW(encoder.setSignalState(3, EncoderSignal::Off), std::invalid_argument);
    EXPECT_THROW(encoder.setSignalState(0, static_cast<EncoderSignal>(7)), std::invalid_argument);
    EXPECT_EQ(encoder.getSignalState(0), EncoderSignal::Nominal);
}

//! The signal states setter rejects a list with a size that is not equal to the wheel count. The encoder keeps its
//! states.
TEST(Encoder, setSignalStatesRejectsWrongSize) {
    Encoder encoder(3, 2);
    encoder.setSignalStates({EncoderSignal::Off, EncoderSignal::Stuck, EncoderSignal::Nominal});

    EXPECT_THROW(encoder.setSignalStates(std::vector<EncoderSignal>(4, EncoderSignal::Off)), std::invalid_argument);
    EXPECT_THROW(encoder.setSignalStates({EncoderSignal::Off}), std::invalid_argument);
    EXPECT_EQ(
        encoder.getSignalStates(),
        (std::vector<EncoderSignal>{EncoderSignal::Off, EncoderSignal::Stuck, EncoderSignal::Nominal})
    );
}

//! Reset keeps the signal states that the user sets before the simulation starts.
TEST(Encoder, resetKeepsConfiguredSignalStates) {
    EncoderHarness harness(3, 2);
    harness.encoder.setSignalStates({EncoderSignal::Off, EncoderSignal::Stuck, EncoderSignal::Nominal});

    harness.encoder.reset(0);

    EXPECT_EQ(harness.encoder.getSignalState(0), EncoderSignal::Off);
    EXPECT_EQ(harness.encoder.getSignalState(1), EncoderSignal::Stuck);
    EXPECT_EQ(harness.encoder.getSignalState(2), EncoderSignal::Nominal);
}

//! The encoder does not measure the wheel angles. It sends the input wheel angles unchanged on each step.
TEST(Encoder, sendsWheelThetasOnEachStep) {
    EncoderHarness harness(3, 2);
    harness.encoder.reset(0);
    harness.step({100.0, 200.0, 300.0}, 0);

    RWSpeedMsgPayload payload{};
    payload.wheelSpeeds[0] = 100.0;
    payload.wheelThetas[0] = 0.25;
    payload.wheelThetas[1] = -1.5;
    payload.wheelThetas[2] = 3.0;
    harness.speedInMsg.write(payload, 0, oneSecond);
    harness.encoder.updateState(oneSecond);
    RWSpeedMsgPayload const out = harness.speedOut();

    EXPECT_DOUBLE_EQ(out.wheelThetas[0], 0.25);
    EXPECT_DOUBLE_EQ(out.wheelThetas[1], -1.5);
    EXPECT_DOUBLE_EQ(out.wheelThetas[2], 3.0);
}

//! The encoder also uses the signal state on a step with a zero time step. An off encoder sends zero speed on the
//! first step.
TEST(Encoder, offSignalAppliesOnFirstStep) {
    EncoderHarness harness(3, 2);
    harness.encoder.setSignalState(1, EncoderSignal::Off);
    harness.encoder.reset(0);

    RWSpeedMsgPayload const out = harness.step({100.0, 200.0, 300.0}, 0);

    EXPECT_DOUBLE_EQ(out.wheelSpeeds[0], 100.0);
    EXPECT_DOUBLE_EQ(out.wheelSpeeds[1], 0.0);
    EXPECT_DOUBLE_EQ(out.wheelSpeeds[2], 300.0);
}

//! When the wheel count decreases, the wheels that the encoder does not read send zero speed. When the wheel count
//! increases again, the count of these wheels starts from zero clicks.
TEST(Encoder, setNumRWSetsUnusedWheelsToZero) {
    EncoderHarness harness(3, 2);
    harness.encoder.reset(0);
    std::vector<double> const speeds{100.0, 200.0, 300.0};
    harness.step(speeds, 0);
    harness.step(speeds, oneSecond);

    harness.encoder.setNumRW(1);
    RWSpeedMsgPayload const reduced = harness.step(speeds, 2 * oneSecond);
    harness.encoder.setNumRW(3);
    RWSpeedMsgPayload const restored = harness.step(speeds, 3 * oneSecond);

    EXPECT_DOUBLE_EQ(reduced.wheelSpeeds[1], 0.0);
    EXPECT_DOUBLE_EQ(reduced.wheelSpeeds[2], 0.0);
    EXPECT_NEAR(restored.wheelSpeeds[1], 63.0 * pi, tolerance);
    EXPECT_NEAR(restored.wheelSpeeds[2], 95.0 * pi, tolerance);
}
