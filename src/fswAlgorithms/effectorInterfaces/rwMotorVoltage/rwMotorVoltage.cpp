// SPDX-License-Identifier: ISC
// Copyright (c) 2016, Autonomous Vehicle System Lab, University of Colorado at Boulder
// Copyright (c) 2025, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

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
