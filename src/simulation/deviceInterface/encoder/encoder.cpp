// SPDX-License-Identifier: ISC
// Copyright (c) 2021, Autonomous Vehicle System Lab, University of Colorado at Boulder
// Copyright (c) 2025, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

#include "encoder.h"

#include <architecture/utilities/macroDefinitions.h>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <iterator>
#include <numbers>
#include <stdexcept>

namespace {
    bool isKnownSignal(EncoderSignal const state) {
        switch (state) {
        case EncoderSignal::Nominal:
        case EncoderSignal::Off:
        case EncoderSignal::Stuck: return true;
        }
        return false;
    }
}  // namespace

Encoder::Encoder(std::size_t const numRW, std::uint32_t const clicksPerRotation) {
    this->setNumRW(numRW);
    this->setClicksPerRotation(clicksPerRotation);
}

void Encoder::reset(uint64_t currentSimNanos) {
    // The input message must be linked.
    if (!this->rwSpeedInMsg.isLinked()) { throw std::invalid_argument("encoder: rwSpeedInMsg is not linked."); }

    // The previous time starts at the reset time.
    this->prevTime = currentSimNanos;

    // The output speeds start at zero.
    this->rwSpeedConverted = RWSpeedMsgPayload{};

    // The remaining clicks start at zero. The signal states do not change.
    for (double &clicks : this->remainingClicks) { clicks = 0.0; }
}

void Encoder::readInputMessages() {
    // The input message gives the wheel speeds and wheel angles of this step.
    this->rwSpeedBuffer = this->rwSpeedInMsg();
}

void Encoder::writeOutputMessages(uint64_t currentClock) {
    this->rwSpeedOutMsg.write(this->rwSpeedConverted, this->moduleID, currentClock);
}

void Encoder::encode(uint64_t currentSimNanos) {
    // This value is the number of clicks in one radian.
    double const clicksPerRadian = static_cast<double>(this->clicksPerRotation) / (2 * std::numbers::pi);

    // The time step is the time since the previous step.
    double const timeStep = (currentSimNanos - this->prevTime) * NANO2SEC;

    // The encoder does not measure the wheel angles. The module sends them unchanged.
    std::copy(
        std::begin(this->rwSpeedBuffer.wheelThetas),
        std::end(this->rwSpeedBuffer.wheelThetas),
        std::begin(this->rwSpeedConverted.wheelThetas)
    );

    // The module calculates the output speed of each wheel that the encoder reads.
    for (std::size_t i = 0; i < this->numRW; i++) {
        switch (this->signalStates[i]) {
        case EncoderSignal::Nominal: {
            // With a zero time step, the encoder cannot count clicks. Thus it sends the input wheel speed.
            if (timeStep == 0.0) {
                this->rwSpeedConverted.wheelSpeeds[i] = this->rwSpeedBuffer.wheelSpeeds[i];
                break;
            }

            // The wheel turns through this angle during the time step.
            double const angle = this->rwSpeedBuffer.wheelSpeeds[i] * timeStep;

            // The encoder counts only an integer number of clicks.
            double const totalClicks = angle * clicksPerRadian + this->remainingClicks[i];
            double const numberClicks = std::trunc(totalClicks);

            // The module keeps the remaining part of a click for the next step.
            this->remainingClicks[i] = totalClicks - numberClicks;

            // The output speed agrees with the number of clicks that the encoder counts.
            this->rwSpeedConverted.wheelSpeeds[i] = numberClicks / (clicksPerRadian * timeStep);
            break;
        }
        case EncoderSignal::Off:
            // An off encoder sends zero speed and erases the remaining part of a click.
            this->rwSpeedConverted.wheelSpeeds[i] = 0.0;
            this->remainingClicks[i] = 0.0;
            break;
        case EncoderSignal::Stuck:
            // A stuck encoder sends the output speed of the previous step.
            break;
        }
    }
}

void Encoder::updateState(uint64_t currentSimNanos) {
    this->readInputMessages();
    this->encode(currentSimNanos);
    this->writeOutputMessages(currentSimNanos);

    this->prevTime = currentSimNanos;
}

void Encoder::setNumRW(std::size_t const numRW) {
    if (numRW == 0) { throw std::invalid_argument("encoder: number of reaction wheels must be more than zero."); }
    if (numRW > RW_EFF_CNT) {
        throw std::invalid_argument("encoder: number of reaction wheels must not be more than RW_EFF_CNT.");
    }
    this->numRW = numRW;

    // The wheels that the encoder does not read send zero speed and have no remaining part of a click.
    for (std::size_t i = numRW; i < RW_EFF_CNT; ++i) {
        this->rwSpeedConverted.wheelSpeeds[i] = 0.0;
        this->remainingClicks[i] = 0.0;
    }
}

std::size_t Encoder::getNumRW() const {
    return this->numRW;
}

void Encoder::setClicksPerRotation(std::uint32_t const clicksPerRotation) {
    if (clicksPerRotation == 0) { throw std::invalid_argument("encoder: clicks per rotation must be more than zero."); }
    this->clicksPerRotation = clicksPerRotation;
}

std::uint32_t Encoder::getClicksPerRotation() const {
    return this->clicksPerRotation;
}

void Encoder::setSignalState(std::size_t const wheel, EncoderSignal const state) {
    if (wheel >= this->numRW) { throw std::invalid_argument("encoder: wheel index must be less than numRW."); }
    if (!isKnownSignal(state)) { throw std::invalid_argument("encoder: signal state is not a known EncoderSignal."); }
    this->signalStates[wheel] = state;
}

EncoderSignal Encoder::getSignalState(std::size_t const wheel) const {
    if (wheel >= this->numRW) { throw std::invalid_argument("encoder: wheel index must be less than numRW."); }
    return this->signalStates[wheel];
}

void Encoder::setSignalStates(std::vector<EncoderSignal> const &states) {
    if (states.size() != this->numRW) {
        throw std::invalid_argument("encoder: number of signal states must be equal to numRW.");
    }
    for (EncoderSignal const state : states) {
        if (!isKnownSignal(state)) {
            throw std::invalid_argument("encoder: signal state is not a known EncoderSignal.");
        }
    }
    std::copy(states.begin(), states.end(), this->signalStates.begin());
}

std::vector<EncoderSignal> Encoder::getSignalStates() const {
    return {this->signalStates.begin(), this->signalStates.begin() + static_cast<std::ptrdiff_t>(this->numRW)};
}
