// SPDX-License-Identifier: ISC
// Copyright (c) 2017, Autonomous Vehicle System Lab, University of Colorado at Boulder
// Copyright (c) 2025, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

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
