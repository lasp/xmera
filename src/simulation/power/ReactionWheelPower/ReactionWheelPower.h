#ifndef XMERA_POWERRW_H
#define XMERA_POWERRW_H

#include <architecture/messaging/messaging.h>
#include <simulation/power/_GeneralModuleFiles/powerNodeBase.h>

#include <architecture/msgPayloadDef/RWConfigLogMsgPayload.h>

#include <architecture/utilities/bskLogging.h>

/*! @brief reaction wheel power class */
class ReactionWheelPower : public PowerNodeBase {
   public:
    ReactionWheelPower();
    ~ReactionWheelPower();
    void customreset(uint64_t currentSimNanos);  //!< Custom reset method
    bool customReadMessages();  //!< Custom read method, similar to customSelfInit; returns `true' by default.

   private:
    void evaluatePowerModel(PowerNodeUsageMsgPayload* powerUsageMsg);

   public:
    ReadFunctor<RWConfigLogMsgPayload> rwStateInMsg;  //!< Reaction wheel state input message name
    double elecToMechEfficiency;  //!< efficiency factor to convert electrical power to mechanical power
    double mechToElecEfficiency;  //!< efficiency factor to convert mechanical power to electrical power
    double basePowerNeed;         //!< [W] base electrical power required to operate RW, typically a positive value
    BSKLogger bskLogger;          //!< -- BSK Logging

   private:
    RWConfigLogMsgPayload rwStatus;  //!< copy of the RW status message
};

#endif  // XMERA_POWERRW_H
