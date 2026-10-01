// SPDX-License-Identifier: ISC
// Copyright (c) 2021, Autonomous Vehicle System Lab, University of Colorado at Boulder
// Copyright (c) 2025, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

#ifndef ENCODER_H
#define ENCODER_H

#include <architecture/_GeneralModuleFiles/sys_model.h>
#include <architecture/messaging/messaging.h>
#include <architecture/msgPayloadDef/RWSpeedMsgPayload.h>

#include <mission/parameters.h>

#include <array>
#include <cstddef>
#include <cstdint>
#include <vector>

/*! @brief Signal state of one wheel encoder. */
enum class EncoderSignal {
    Nominal,  //!< The encoder operates correctly.
    Off,      //!< The encoder sends zero speed.
    Stuck     //!< The encoder sends the speed of the previous step.
};

/*! @brief Reaction wheel speed encoder.

    The encoder counts the clicks of each wheel in each time step. It sends the wheel speed that agrees with that
    count. It also simulates encoder failures with a signal state for each wheel. */
class Encoder : public SysModel {
public:
    /*! @brief Makes an encoder for the given wheel count and resolution. All signal states are nominal.
        @param numRW number of reaction wheels, from one to RW_EFF_CNT.
        @param clicksPerRotation number of clicks in one rotation. Zero is not permitted.
        @throws std::invalid_argument if a parameter is not in its permitted range. */
    Encoder(std::size_t numRW, std::uint32_t clicksPerRotation);

    /*! @brief Sets the remaining clicks and the output speeds to zero. The signal states do not change.
        @param currentSimNanos [ns] simulation time
        @throws std::invalid_argument if rwSpeedInMsg is not linked. */
    void reset(uint64_t currentSimNanos) override;
    /*! @brief Reads the input message, calculates the encoder output, and writes the output message.
        @param currentSimNanos [ns] simulation time */
    void updateState(uint64_t currentSimNanos) override;
    /*! @brief Reads the reaction wheel speed input message. */
    void readInputMessages();
    /*! @brief Writes the encoder output speeds to the output message.
        @param currentClock [ns] simulation time */
    void writeOutputMessages(uint64_t currentClock);
    /*! @brief Calculates the encoder output speeds from the input speeds and the signal states.
        @param currentSimNanos [ns] simulation time */
    void encode(uint64_t currentSimNanos);

    /*! @brief Sets the number of reaction wheels that the encoder reads. The output speeds and the remaining clicks of
        the other wheels change to zero.
        @param numRW number of reaction wheels, from one to RW_EFF_CNT.
        @throws std::invalid_argument if numRW is zero or more than RW_EFF_CNT. */
    void setNumRW(std::size_t numRW);
    /*! @brief Gets the number of reaction wheels that the encoder reads. */
    std::size_t getNumRW() const;
    /*! @brief Sets the number of encoder clicks in one full wheel rotation.
        @param clicksPerRotation number of clicks in one rotation. Zero is not permitted.
        @throws std::invalid_argument if clicksPerRotation is zero. */
    void setClicksPerRotation(std::uint32_t clicksPerRotation);
    /*! @brief Gets the number of encoder clicks in one full wheel rotation. */
    std::uint32_t getClicksPerRotation() const;
    /*! @brief Sets the signal state of one wheel encoder.
        @param wheel index of the reaction wheel, less than the wheel count.
        @param state signal state of the encoder.
        @throws std::invalid_argument if the wheel index or the state is incorrect. */
    void setSignalState(std::size_t wheel, EncoderSignal state);
    /*! @brief Gets the signal state of one wheel encoder.
        @param wheel index of the reaction wheel, less than the wheel count.
        @throws std::invalid_argument if the wheel index is incorrect. */
    EncoderSignal getSignalState(std::size_t wheel) const;
    /*! @brief Sets the signal states of all wheel encoders.
        @param states one signal state for each reaction wheel. The size must be equal to the wheel count.
        @throws std::invalid_argument if the size or a state is incorrect. The encoder keeps its states. */
    void setSignalStates(std::vector<EncoderSignal> const &states);
    /*! @brief Gets the signal states of all wheel encoders, one for each reaction wheel. */
    std::vector<EncoderSignal> getSignalStates() const;

public:
    Message<RWSpeedMsgPayload> rwSpeedOutMsg;     //!< [rad/s] encoder output wheel speeds
    ReadFunctor<RWSpeedMsgPayload> rwSpeedInMsg;  //!< [rad/s] input wheel speeds

private:
    std::size_t numRW = 0;                                 //!< number of reaction wheels
    std::uint32_t clicksPerRotation = 0;                   //!< number of clicks per full rotation
    std::array<EncoderSignal, RW_EFF_CNT> signalStates{};  //!< signal state of each wheel encoder
    RWSpeedMsgPayload rwSpeedBuffer{};                     //!< [rad/s] input wheel speeds of this step
    RWSpeedMsgPayload rwSpeedConverted{};                  //!< [rad/s] encoder output wheel speeds
    double remainingClicks[RW_EFF_CNT]{};                  //!< remaining part of a click from the previous step

    uint64_t prevTime = 0;  //!< [ns] simulation time of the previous step
};

#endif
