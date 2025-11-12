#ifndef RW_MOTOR_VOLTAGE_H
#define RW_MOTOR_VOLTAGE_H

#include <stdint.h>

#include <architecture/_GeneralModuleFiles/sys_model.h>
#include <architecture/messaging/messaging.h>
#include <architecture/msgPayloadDef/RWArrayConfigMsgPayload.h>
#include <architecture/msgPayloadDef/RWAvailabilityMsgPayload.h>
#include <architecture/msgPayloadDef/RWSpeedMsgPayload.h>
#include <architecture/msgPayloadDef/RwMotorTorqueMsgPayload.h>
#include <architecture/msgPayloadDef/RwMotorVoltageMsgPayload.h>
#include "rwMotorVoltageAlgorithm.h"

#include <Eigen/Core>

class RwMotorVoltage : public SysModel {
   public:
    RwMotorVoltage(const double minVoltageMagnitude, const double maxVoltageMagnitude);
    ~RwMotorVoltage() final = default;

    void reset(uint64_t callTime) override;
    void updateState(uint64_t callTime) override;

    void setVoltageRange(const double minVoltageMagnitude, const double maxVoltageMagnitude);
    Eigen::Vector2d getVoltageRange() const;
    void setGainK(const double gain);
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
