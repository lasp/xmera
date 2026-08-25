// SPDX-License-Identifier: ISC
// Copyright (c) 2017, Autonomous Vehicle System Lab, University of Colorado at Boulder
// Copyright (c) 2025, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

#ifndef RW_MOTOR_VOLTAGE_H
#define RW_MOTOR_VOLTAGE_H

#include "rwMotorVoltageAlgorithm.h"
#include <architecture/_GeneralModuleFiles/sys_model.h>
#include <architecture/messaging/messaging.h>
#include <architecture/msgPayloadDef/RWArrayConfigMsgPayload.h>
#include <architecture/msgPayloadDef/RWAvailabilityMsgPayload.h>
#include <architecture/msgPayloadDef/RwMotorTorqueMsgPayload.h>
#include <architecture/msgPayloadDef/RwMotorVoltageMsgPayload.h>
#include <architecture/msgPayloadDef/RWSpeedMsgPayload.h>

#include <stdint.h>

#include <Eigen/Core>

class RwMotorVoltage final : public SysModel {
public:
    RwMotorVoltage(double const minVoltageMagnitude, double const maxVoltageMagnitude);

    void reset(uint64_t callTime) override;
    void updateState(uint64_t callTime) override;

    void setVoltageRange(double const minVoltageMagnitude, double const maxVoltageMagnitude);
    Eigen::Vector2d getVoltageRange() const;
    void setGainK(double const gain);
    double getGainK() const;

    /* declare module IO interfaces */
    Message<RwMotorVoltageMsgPayload> voltageOutMsg;    /*!< voltage output message*/
    ReadFunctor<RwMotorTorqueMsgPayload> torqueInMsg;   /*!< Input torque message*/
    ReadFunctor<RWArrayConfigMsgPayload> rwParamsInMsg; /*!< RW array input message*/
    ReadFunctor<RWSpeedMsgPayload> rwSpeedInMsg;        /*!< [] The name for the reaction wheel speeds message. Must be
                                                           provided to enable speed tracking loop */
    ReadFunctor<RWAvailabilityMsgPayload> rwAvailInMsg; /*!< [-] The name of the RWs availability message*/

private:
    RwMotorVoltageAlgorithm algorithm;
};

#endif
