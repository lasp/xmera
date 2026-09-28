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

/*! This method is used to reset the module.
 @return void
 */
void Encoder::reset(uint64_t currentSimNanos) {
    // check if input message is linked
    if (!this->rwSpeedInMsg.isLinked()) { throw std::invalid_argument("encoder: rwSpeedInMsg is not linked."); }

    // reset the previous time
    this->prevTime = currentSimNanos;

    // zero the RW wheel output message buffer //
    this->rwSpeedConverted = RWSpeedMsgPayload{};

    // set the remaining clicks to zero, and keep the configured signal states
    for (double &clicks : this->remainingClicks) { clicks = 0.0; }
}

/*! This method reads the speed input message
 */
void Encoder::readInputMessages() {
    // read the incoming wheel speed message
    this->rwSpeedBuffer = this->rwSpeedInMsg();
}

/*! This method writes encoded the wheel speed message.
 @return void
 @param CurrentClock The clock time associated with the model call
 */
void Encoder::writeOutputMessages(uint64_t CurrentClock) {
    this->rwSpeedOutMsg.write(this->rwSpeedConverted, this->moduleID, CurrentClock);
}

/*! This method applies an encoder to the reaction wheel speeds.
 */
void Encoder::encode(uint64_t currentSimNanos) {
    // convert clicks per rotation to clicks per radian
    double const clicksPerRadian = static_cast<double>(this->clicksPerRotation) / (2 * std::numbers::pi);

    // set the time step
    double const timeStep = (currentSimNanos - this->prevTime) * NANO2SEC;

    // the encoder does not measure the wheel angles, so send them unchanged
    std::copy(
        std::begin(this->rwSpeedBuffer.wheelThetas),
        std::end(this->rwSpeedBuffer.wheelThetas),
        std::begin(this->rwSpeedConverted.wheelThetas)
    );

    // loop through the RW
    for (std::size_t i = 0; i < this->numRW; i++) {
        switch (this->signalStates[i]) {
        case EncoderSignal::Nominal: {
            // with a zero time step there are no clicks to count, so the encoder outputs the true RW speed
            if (timeStep == 0.0) {
                this->rwSpeedConverted.wheelSpeeds[i] = this->rwSpeedBuffer.wheelSpeeds[i];
                break;
            }

            // calculate the angle sweeped by the reaction wheel during the time step
            double const angle = this->rwSpeedBuffer.wheelSpeeds[i] * timeStep;

            // calculate the number of clicks
            double const totalClicks = angle * clicksPerRadian + this->remainingClicks[i];
            double const numberClicks = std::trunc(totalClicks);

            // update the remaining clicks
            this->remainingClicks[i] = totalClicks - numberClicks;

            // calculate the discretized angular velocity
            this->rwSpeedConverted.wheelSpeeds[i] = numberClicks / (clicksPerRadian * timeStep);
            break;
        }
        case EncoderSignal::Off:
            // set the outgoing reaction wheel speed to 0 and reset the remaining clicks
            this->rwSpeedConverted.wheelSpeeds[i] = 0.0;
            this->remainingClicks[i] = 0.0;
            break;
        case EncoderSignal::Stuck:
            // if the encoder is stuck, it will output the previous results
            break;
        }
    }
}

/*! This method runs the encoder module in the sim.
 */
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
