// SPDX-License-Identifier: ISC
// Copyright (c) 2026, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

#ifndef ENCODER_TEST_HELPERS_HPP
#define ENCODER_TEST_HELPERS_HPP

#include <architecture/messaging/messaging.h>
#include <architecture/msgPayloadDef/RWSpeedMsgPayload.h>

#include <simulation/deviceInterface/encoder/encoder.h>

#include <cstddef>
#include <cstdint>
#include <vector>

namespace encodertest {
    //! One second in nanoseconds. The tests use this value as the time step.
    constexpr uint64_t oneSecond = 1'000'000'000ULL;

    //! This harness connects an encoder to an input speed message and an output reader.
    class EncoderHarness {
    public:
        //! Make an encoder with the given wheel count and clicks per rotation, then connect its messages.
        EncoderHarness(std::size_t numRW, std::uint32_t clicksPerRotation) {
            this->encoder.setNumRW(numRW);
            this->encoder.setClicksPerRotation(clicksPerRotation);
            this->encoder.rwSpeedInMsg.subscribeTo(&this->speedInMsg);
            this->speedOut = this->encoder.rwSpeedOutMsg.addSubscriber();
        }

        EncoderHarness(EncoderHarness const &) = delete;
        EncoderHarness &operator=(EncoderHarness const &) = delete;

        //! Write the wheel speeds to the input message, update the encoder at time t, and read the output.
        RWSpeedMsgPayload step(std::vector<double> const &wheelSpeeds, uint64_t t) {
            RWSpeedMsgPayload payload{};
            for (std::size_t i = 0; i < wheelSpeeds.size(); ++i) { payload.wheelSpeeds[i] = wheelSpeeds[i]; }
            this->speedInMsg.write(payload, 0, t);
            this->encoder.updateState(t);
            return this->speedOut();
        }

        Encoder encoder;                          //!< encoder under test
        Message<RWSpeedMsgPayload> speedInMsg;    //!< input wheel speed message
        ReadFunctor<RWSpeedMsgPayload> speedOut;  //!< reader of the encoder output message
    };
}  // namespace encodertest

#endif  // ENCODER_TEST_HELPERS_HPP
