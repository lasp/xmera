// SPDX-License-Identifier: ISC
// Copyright (c) 2021, Autonomous Vehicle System Lab, University of Colorado at Boulder
// Copyright (c) 2025, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

#ifndef ENCODER_H
#define ENCODER_H

#include <architecture/_GeneralModuleFiles/sys_model.h>
#include <architecture/messaging/messaging.h>
#include <architecture/msgPayloadDef/RWSpeedMsgPayload.h>
#include <architecture/utilities/bskLogging.h>

#include <mission/parameters.h>

#include <cstddef>
#include <cstdint>

/*! @brief Signal state of one wheel encoder. */
enum class EncoderSignal {
    Nominal,  //!< The encoder operates correctly.
    Off,      //!< The encoder sends zero speed.
    Stuck     //!< The encoder sends the speed of the previous step.
};

/*! @brief wheel speed encoder module class */
class Encoder : public SysModel {
public:
    /*! @brief Make an encoder for the given wheel count and resolution.
        @param numRW number of reaction wheels, from one to RW_EFF_CNT.
        @param clicksPerRotation number of clicks in one rotation. Zero is not permitted.
        @throws std::invalid_argument if a parameter is not in its permitted range. */
    Encoder(std::size_t numRW, std::uint32_t clicksPerRotation);

    void reset(uint64_t currentSimNanos) override;
    void updateState(uint64_t currentSimNanos) override;
    void readInputMessages();
    void writeOutputMessages(uint64_t CurrentClock);
    void encode(uint64_t currentSimNanos);

    /*! @brief Set the number of reaction wheels that the encoder reads.
        @param numRW number of reaction wheels, from one to RW_EFF_CNT.
        @throws std::invalid_argument if numRW is zero or more than RW_EFF_CNT. */
    void setNumRW(std::size_t numRW);
    /*! @brief Get the number of reaction wheels that the encoder reads. */
    std::size_t getNumRW() const;
    /*! @brief Set the number of encoder clicks in one full wheel rotation.
        @param clicksPerRotation number of clicks in one rotation. Zero is not permitted.
        @throws std::invalid_argument if clicksPerRotation is zero. */
    void setClicksPerRotation(std::uint32_t clicksPerRotation);
    /*! @brief Get the number of encoder clicks in one full wheel rotation. */
    std::uint32_t getClicksPerRotation() const;

public:
    Message<RWSpeedMsgPayload> rwSpeedOutMsg;     //!< [rad/s] reaction wheel speed output message
    ReadFunctor<RWSpeedMsgPayload> rwSpeedInMsg;  //!< [rad/s] reaction wheel speed input message
    EncoderSignal rwSignalState[RW_EFF_CNT]{};    //!< vector of reaction wheel signal states
    BSKLogger bskLogger;                          //!< -- BSK Logging

private:
    std::size_t numRW = 0;                 //!< number of reaction wheels
    std::uint32_t clicksPerRotation = 0;   //!< number of clicks per full rotation
    RWSpeedMsgPayload rwSpeedBuffer{};     //!< reaction wheel speed buffer for internal calculations
    RWSpeedMsgPayload rwSpeedConverted{};  //!< reaction wheel speed buffer for converted values
    double remainingClicks[RW_EFF_CNT]{};  //!< remaining clicks from the previous iteration

    uint64_t prevTime = 0;  //!< -- Previous simulation time observed
};

#endif
