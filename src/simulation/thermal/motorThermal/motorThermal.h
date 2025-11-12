#ifndef MOTOR_THERMAL_H
#define MOTOR_THERMAL_H

#include <architecture/_GeneralModuleFiles/sys_model.h>
#include <architecture/messaging/messaging.h>
#include <architecture/msgPayloadDef/RWConfigLogMsgPayload.h>
#include <architecture/msgPayloadDef/TemperatureMsgPayload.h>
#include <architecture/utilities/bskLogging.h>

/*! @brief Motor temperature module.  It simulates the heating and cooling of a motor based on ambient temperature, as
 * well as heat generated during spin-up or breaking. */
class MotorThermal : public SysModel {
   public:
    MotorThermal();
    ~MotorThermal();

    void reset(uint64_t currentSimNanos);
    void updateState(uint64_t currentSimNanos);
    void readInputMessages();
    void writeOutputMessages(uint64_t CurrentClock);
    void computeTemperature(uint64_t currentSimNanos);

   public:
    Message<TemperatureMsgPayload> temperatureOutMsg;  //!< [Celsius] temperature output message
    ReadFunctor<RWConfigLogMsgPayload> rwStateInMsg;   //!< reaction wheel state input message
    double efficiency;                                 //!< efficiency factor to convert power into mechanical power
    double currentTemperature;                         //!< [Celsius] stored temperature
    double ambientTemperature;                         //!< [Celsius] ambient temperature for heat dissipation
    double ambientThermalResistance;  //!< [W/Celsius] ambient thermal resistance to convert heat into temperature
    double motorHeatCapacity;         //!< [J/Celsius] motor heat caapcity to convert heat into temperature
    BSKLogger bskLogger;              //!< -- BSK Logging

   private:
    TemperatureMsgPayload temperatureBuffer;  //!< temperature buffer for internal calculations
    RWConfigLogMsgPayload rwStateBuffer;      //!< reaction wheel state buffer

    uint64_t prevTime;  //!< -- Previous simulation time observed
};

#endif
