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

#include "rwMotorVoltageAlgorithm.h"
#include <architecture/utilities/macroDefinitions.h>

#include <ranges>
#include <stdexcept>

/**
 * @brief Construct RwMotorVoltageAlgorithm
 * @param minVoltageMagnitude minimum voltage below which the torque is zero.
 * @param maxVoltageMagnitude maximum output voltage
 */
RwMotorVoltageAlgorithm::RwMotorVoltageAlgorithm(const double minVoltageMagnitude, const double maxVoltageMagnitude) {
    this->setVoltageRange(minVoltageMagnitude, maxVoltageMagnitude);
}

/*! This method performs a reset of the module as far as closed loop control is concerned.  Local module variables that
 retain time varying states between function calls are reset to their default values.
 @return void
 @param rwParamsInMsg struct to store message containing RW config parameters
 */
void RwMotorVoltageAlgorithm::reset(RWArrayConfigMsgPayload& rwParamsInMsg) {
    /*! - Read static RW config data message and store it in module variables*/
    this->rwConfigParams = rwParamsInMsg;

    /* reset variables */
    this->rwSpeedOld.setZero();
    this->resetFlag = true;

    /* Reset the prior time flag state.
     If zero, control time step not evaluated on the first function call */
    this->priorTime = 0;
}

/*! Update performs the torque to voltage conversion. If a wheel speed message was provided, it also does closed loop
 control of the voltage sent. It then writes the voltage message.
 @return void
 @param callTime The clock time at which the function was called (nanoseconds)
 @param torqueCmd RW motor torque input message
 @param rwAvailability RW availability message
 @param rwSpeed RW speed message
 @param rwSpeedMsgIsLinked boolean indicating whether RW speed message is linked
 */
RwMotorVoltageMsgPayload RwMotorVoltageAlgorithm::update(uint64_t callTime,
                                                         RwMotorTorqueMsgPayload& torqueCmd,
                                                         RWAvailabilityMsgPayload& rwAvailability,
                                                         RWSpeedMsgPayload& rwSpeed,
                                                         bool rwSpeedMsgIsLinked) {
    RwMotorVoltageMsgPayload voltageOut{};

    /* zero the output voltage vector */
    Eigen::Vector<double, RW_EFF_CNT> voltage{};
    voltage.setZero();

    /* if the torque closed-loop is on, evaluate the feedback term */
    if (rwSpeedMsgIsLinked) {
        /* make sure the clock didn't just initialize, or the module was recently reset */
        if (this->priorTime != 0) {
            double dt = (callTime - this->priorTime) * NANO2SEC; /*!< [s]   control update period */
            Eigen::Vector<double, RW_EFF_CNT> OmegaDot{};
            OmegaDot.setZero();
            for (int i = 0; i < this->rwConfigParams.numRW; i++) {
                if (rwAvailability.wheelAvailability[i] == AVAILABLE && this->resetFlag == false) {
                    OmegaDot[i] = (rwSpeed.wheelSpeeds[i] - this->rwSpeedOld[i]) / dt;
                    torqueCmd.motorTorque[i] -=
                        this->K * (this->rwConfigParams.JsList[i] * OmegaDot[i] - torqueCmd.motorTorque[i]);
                }
                this->rwSpeedOld[i] = rwSpeed.wheelSpeeds[i];
            }
            this->resetFlag = false;
        }
        this->priorTime = callTime;
    }

    /* evaluate the feedforward mapping of torque into voltage */
    for (int i = 0; i < this->rwConfigParams.numRW; ++i) {
        if (rwAvailability.wheelAvailability[i] == AVAILABLE) {
            voltage[i] =
                (this->voltageMax - this->voltageMin) / this->rwConfigParams.uMax[i] * torqueCmd.motorTorque[i];
            if (voltage[i] > 0.0) voltage[i] += this->voltageMin;
            if (voltage[i] < 0.0) voltage[i] -= this->voltageMin;
        }
        /* check for voltage saturation */
        voltage[i] = std::ranges::clamp(voltage[i], -this->voltageMax, this->voltageMax);

        voltageOut.voltage[i] = voltage[i];
    }

    return voltageOut;
}

/**
 * @brief Set the minimum and maximum voltage.
 * @param minVoltageMagnitude minimum voltage below which the torque is zero.
 * @param maxVoltageMagnitude maximum output voltage
 */
void RwMotorVoltageAlgorithm::setVoltageRange(const double minVoltageMagnitude, const double maxVoltageMagnitude) {
    if (minVoltageMagnitude < 0.0) {
        throw std::invalid_argument("minVoltageMagnitude must not be negative.");
    }
    if (maxVoltageMagnitude < 0.0) {
        throw std::invalid_argument("maxVoltageMagnitude must not be negative.");
    }
    if (maxVoltageMagnitude <= minVoltageMagnitude) {
        throw std::invalid_argument("maxVoltageMagnitude must be greater than minVoltageMagnitude.");
    }
    this->voltageMin = minVoltageMagnitude;
    this->voltageMax = maxVoltageMagnitude;
}

/**
 * @brief Get the minimum and maximum voltage.
 * @return Eigen::Vector2d minimum and maximum voltage
 */
Eigen::Vector2d RwMotorVoltageAlgorithm::getVoltageRange() const {
    return Eigen::Vector2d{this->voltageMin, this->voltageMax};
}

/**
 * @brief Set the feedback gain.
 * @param gain feedback gain.
 */
void RwMotorVoltageAlgorithm::setGainK(const double gain) {
    if (gain < 0.0) {
        throw std::invalid_argument("Feedback gain must not be negative");
    }
    this->K = gain;
}

/**
 * @brief Get the feedback gain.
 * @return double feedback gain.
 */
double RwMotorVoltageAlgorithm::getGainK() const { return this->K; }
