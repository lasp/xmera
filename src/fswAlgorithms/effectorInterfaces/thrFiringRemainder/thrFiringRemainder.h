#ifndef _THR_FIRING_REMAINDER_
#define _THR_FIRING_REMAINDER_

#include <stdint.h>

#include <architecture/_GeneralModuleFiles/sys_model.h>
#include <architecture/messaging/messaging.h>
#include <architecture/msgPayloadDef/THRArrayCmdForceMsgPayload.h>
#include <architecture/msgPayloadDef/THRArrayConfigMsgPayload.h>
#include <architecture/msgPayloadDef/THRArrayOnTimeCmdMsgPayload.h>

#include <architecture/msgPayloadDef/definitions.h>
#include <architecture/utilities/bskLogging.h>

/*! @brief Top level structure for the sub-module routines. */
class ThrFiringRemainder : public SysModel {
   public:
    void reset(uint64_t callTime) override;
    void updateState(uint64_t callTime) override;

    void setThrMinFireTime(double thrMinFireTime);  //!< Setter for thrMinFireTime variable
    double getThrMinFireTime() const;              //!< Getter for thrMinFireTime variable

    void setBaseThrustState(int baseThrustState);  //!< Setter for baseThrustState variable
    int getBaseThrustState() const;               //!< Getter for baseThrustState variable

    void setDefaultControlPeriod(double defaultControlPeriod);  //!< Setter for defaultControlPeriod variable
    double getDefaultControlPeriod() const;                    //!< Getter for defaultControlPeriod variable

    /* declare module IO interfaces */
    ReadFunctor<THRArrayCmdForceMsgPayload> thrForceInMsg;  //!< The name of the Input message
    Message<THRArrayOnTimeCmdMsgPayload> onTimeOutMsg;      //!< The name of the output message, onTimeOutMsg
    ReadFunctor<THRArrayConfigMsgPayload> thrConfInMsg;     //!< The name of the thruster cluster Input message
    BSKLogger bskLogger = {};                               //!< BSK Logging

   private:
    double pulseRemainder[MAX_EFF_CNT]{};  //!< [-] Unimplemented thrust pulses (number of minimum pulses)
    double thrMinFireTime{};               //!< [s] Minimum fire time
    int numThrusters{};                    //!< [-] The number of thrusters available on vehicle
    double maxThrust[MAX_EFF_CNT]{};       //!< [N] Max thrust
    int baseThrustState{};                 //!< [-] Indicates on-pulsing (0) or off-pulsing (1)
    double defaultControlPeriod{};         //!< [s] Default control period used for first call
    uint64_t prevCallTime{};               //!< callTime from previous function call
};

#endif
