// SPDX-License-Identifier: ISC
// Copyright (c) 2026, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

#include "encoderTestHelpers.hpp"

#include <fuzztest/fuzztest.h>
#include <gtest/gtest.h>

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <numbers>
#include <tuple>
#include <utility>
#include <vector>

namespace {
    using encodertest::EncoderHarness;

    constexpr std::size_t maxFuzzWheels = 4;
    constexpr std::size_t maxFuzzSteps = 50;
    constexpr double maxWheelSpeed = 1000.0;             // [rad/s]
    constexpr uint64_t minTimeStep = 1'000'000ULL;       // [ns] 1 ms
    constexpr uint64_t maxTimeStep = 10'000'000'000ULL;  // [ns] 10 s

    //! One step of fuzz input: the speed of all wheels and the time step in nanoseconds.
    using SpeedStep = std::pair<double, uint64_t>;

    auto speedSteps() {
        return fuzztest::VectorOf(
                   fuzztest::PairOf(
                       fuzztest::InRange(-maxWheelSpeed, maxWheelSpeed),
                       fuzztest::InRange(minTimeStep, maxTimeStep)
                   )
        )
            .WithMaxSize(maxFuzzSteps);
    }

    //! For nominal encoders, the angle that the encoder output shows and the true wheel angle differ by less than one
    //! click. The encoder keeps the remaining part of a click for the next step, so the error does not increase.
    void angleErrorStaysBelowOneClick(
        std::size_t numRW,
        std::uint32_t clicksPerRotation,
        std::vector<SpeedStep> const &steps
    ) {
        EncoderHarness harness(numRW, clicksPerRotation);
        harness.encoder.reset(0);
        harness.step(std::vector<double>(numRW, 0.0), 0);

        double const clickAngle = 2.0 * std::numbers::pi / clicksPerRotation;
        std::vector<double> trueAngle(numRW, 0.0);
        std::vector<double> encodedAngle(numRW, 0.0);
        double angleScale = 1.0;
        uint64_t t = 0;
        for (auto const &[speed, timeStepNanos] : steps) {
            t += timeStepNanos;
            double const timeStep = static_cast<double>(timeStepNanos) * 1.0e-9;
            RWSpeedMsgPayload const out = harness.step(std::vector<double>(numRW, speed), t);
            angleScale += std::abs(speed * timeStep);
            for (std::size_t i = 0; i < numRW; ++i) {
                ASSERT_TRUE(std::isfinite(out.wheelSpeeds[i]));
                trueAngle[i] += speed * timeStep;
                encodedAngle[i] += out.wheelSpeeds[i] * timeStep;
                EXPECT_LT(std::abs(encodedAngle[i] - trueAngle[i]), clickAngle + 1.0e-12 * angleScale);
            }
        }
    }

    FUZZ_TEST(EncoderFuzz, angleErrorStaysBelowOneClick)
        .WithDomains(
            fuzztest::InRange<std::size_t>(1, maxFuzzWheels),
            fuzztest::InRange<std::uint32_t>(1, 4'096),
            speedSteps()
        );

    //! One step of fuzz input for the signal state property: the speed of all wheels, the time step in nanoseconds,
    //! and the signal state of each wheel as an integer.
    using SignalStep = std::tuple<double, uint64_t, std::vector<int>>;

    //! An off encoder sends zero speed. A stuck encoder sends the speed of the previous step. All output speeds are
    //! finite for all sequences of signal states.
    void signalStatesHaveTheirEffect(std::vector<SignalStep> const &steps) {
        std::size_t const numRW = maxFuzzWheels;
        EncoderHarness harness(numRW, 64);
        harness.encoder.reset(0);
        RWSpeedMsgPayload previous = harness.step(std::vector<double>(numRW, 0.0), 0);

        uint64_t t = 0;
        for (auto const &[speed, timeStepNanos, states] : steps) {
            t += timeStepNanos;
            for (std::size_t i = 0; i < numRW; ++i) {
                harness.encoder.setSignalState(i, static_cast<EncoderSignal>(states[i]));
            }
            RWSpeedMsgPayload const out = harness.step(std::vector<double>(numRW, speed), t);
            for (std::size_t i = 0; i < numRW; ++i) {
                ASSERT_TRUE(std::isfinite(out.wheelSpeeds[i]));
                if (states[i] == static_cast<int>(EncoderSignal::Off)) { EXPECT_EQ(out.wheelSpeeds[i], 0.0); }
                if (states[i] == static_cast<int>(EncoderSignal::Stuck)) {
                    EXPECT_EQ(out.wheelSpeeds[i], previous.wheelSpeeds[i]);
                }
            }
            previous = out;
        }
    }

    FUZZ_TEST(EncoderFuzz, signalStatesHaveTheirEffect)
        .WithDomains(
            fuzztest::VectorOf(
                fuzztest::TupleOf(
                    fuzztest::InRange(-maxWheelSpeed, maxWheelSpeed),
                    fuzztest::InRange(minTimeStep, maxTimeStep),
                    fuzztest::VectorOf(fuzztest::InRange(0, 2)).WithSize(maxFuzzWheels)
                )
            )
                .WithMaxSize(maxFuzzSteps)
        );
}  // namespace
