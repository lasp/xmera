/*
 ISC License

 Copyright (c) 2025, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

 Permission to use, copy, modify, and/or distribute this software for any
 purpose with or without fee is hereby granted, provided that the above
 copyright notice and this permission notice appear in all copies.

 THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
 WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
 MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
 ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
 WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
 ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
 OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.

 */

#include "rwMotorVoltage.h"

#include <stdexcept>

/**
 * @brief Construct RwMotorVoltage
 * @param minVoltageMagnitude minimum voltage below which the torque is zero.
 * @param maxVoltageMagnitude maximum output voltage
 */
RwMotorVoltage::RwMotorVoltage(const double minVoltageMagnitude, const double maxVoltageMagnitude)
    : algorithm(minVoltageMagnitude, maxVoltageMagnitude) {}

/*! This method performs a reset of the module as far as closed loop control is concerned.  Local module variables that
 retain time varying states between function calls are reset to their default values.
 @return void
 @param callTime Sim time in nanos
 */
void RwMotorVoltage::reset(uint64_t callTime) {
    // check if the required input messages are included
    if (!this->rwParamsInMsg.isLinked()) {
        throw std::invalid_argument("rwMotorVoltage.rwParamsInMsg wasn't connected.");
    }
    if (!this->torqueInMsg.isLinked()) {
        throw std::invalid_argument("rwMotorVoltage.torqueInMsg wasn't connected.");
    }

    RWArrayConfigMsgPayload rwParams = this->rwParamsInMsg();

    this->algorithm.reset(rwParams);
}

/*! Update performs the torque to voltage conversion. If a wheel speed message was provided, it also does closed loop
 control of the voltage sent. It then writes the voltage message.
 @return void
 @param callTime The clock time at which the function was called (nanoseconds)
 */
void RwMotorVoltage::updateState(uint64_t callTime) {
    /* - Read the input messages */
    RwMotorTorqueMsgPayload torqueCmd = this->torqueInMsg(); /*!< copy of RW motor torque input message*/
    RWSpeedMsgPayload rwSpeed{};                             /*!< [r/s] Reaction wheel speed estimates */
    RWAvailabilityMsgPayload rwAvailability{};

    bool rwSpeedMsgIsLinked{};
    if (this->rwSpeedInMsg.isLinked()) {
        rwSpeed = this->rwSpeedInMsg();
        rwSpeedMsgIsLinked = true;
    }
    if (this->rwAvailInMsg.isLinked()) {
        rwAvailability = this->rwAvailInMsg();
    }

    RwMotorVoltageMsgPayload voltageOut =
        this->algorithm.update(callTime, torqueCmd, rwAvailability, rwSpeed, rwSpeedMsgIsLinked);

    this->voltageOutMsg.write(&voltageOut, this->moduleID, callTime);
}

/**
 * @brief Set the minimum and maximum voltage.
 * @param minVoltageMagnitude minimum voltage below which the torque is zero.
 * @param maxVoltageMagnitude maximum output voltage
 */
void RwMotorVoltage::setVoltageRange(const double minVoltageMagnitude, const double maxVoltageMagnitude) {
    this->algorithm.setVoltageRange(minVoltageMagnitude, maxVoltageMagnitude);
}

/**
 * @brief Get the minimum and maximum voltage.
 * @return Eigen::Vector2d minimum and maximum voltage
 */
Eigen::Vector2d RwMotorVoltage::getVoltageRange() const { return this->algorithm.getVoltageRange(); }

/**
 * @brief Set the feedback gain.
 * @param gain feedback gain.
 */
void RwMotorVoltage::setGainK(const double gain) { this->algorithm.setGainK(gain); }

/**
 * @brief Get the feedback gain.
 * @return double feedback gain.
 */
double RwMotorVoltage::getGainK() const { return this->algorithm.getGainK(); }
