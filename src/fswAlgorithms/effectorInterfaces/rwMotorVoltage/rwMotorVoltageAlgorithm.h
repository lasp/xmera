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

#ifndef RW_MOTOR_VOLTAGE_ALGORITHM_H
#define RW_MOTOR_VOLTAGE_ALGORITHM_H

#include <stdint.h>

#include <architecture/msgPayloadDef/RWArrayConfigMsgPayload.h>
#include <architecture/msgPayloadDef/RWAvailabilityMsgPayload.h>
#include <architecture/msgPayloadDef/RWSpeedMsgPayload.h>
#include <architecture/msgPayloadDef/RwMotorTorqueMsgPayload.h>
#include <architecture/msgPayloadDef/RwMotorVoltageMsgPayload.h>

#include <Eigen/Core>

/*! @brief module configuration message */
class RwMotorVoltageAlgorithm {
   public:
    RwMotorVoltageAlgorithm(const double minVoltageMagnitude, const double maxVoltageMagnitude);
    ~RwMotorVoltageAlgorithm() = default;

    void reset(RWArrayConfigMsgPayload& rwParamsInMsg);
    RwMotorVoltageMsgPayload update(uint64_t callTime,
                                    RwMotorTorqueMsgPayload& torqueCmd,
                                    RWAvailabilityMsgPayload& rwAvailability,
                                    RWSpeedMsgPayload& rwSpeed,
                                    bool rwSpeedMsgIsLinked);

    void setVoltageRange(const double minVoltageMagnitude, const double maxVoltageMagnitude);
    Eigen::Vector2d getVoltageRange() const;
    void setGainK(const double gain);
    double getGainK() const;

   private:
    double voltageMin{};                            /*!< [V]    minimum voltage below which the torque is zero */
    double voltageMax{};                            /*!< [V]    maximum output voltage */
    double K{};                                     /*!< [V/Nm] torque tracking gain for closed loop control.*/
    Eigen::Vector<double, RW_EFF_CNT> rwSpeedOld{}; /*!< [r/s]  the RW spin rates from the prior control step */
    uint64_t priorTime{};                           /*!< [ns]   Last time the module control was called */
    bool resetFlag{};                               /*!< []     Flag indicating that a module reset occurred */
    RWArrayConfigMsgPayload
        rwConfigParams{}; /*!< [-] struct to store message containing RW config parameters in body B frame */
};

#endif
