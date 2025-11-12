#ifndef XMERA_THRFIRINGSCHMITT_H
#define XMERA_THRFIRINGSCHMITT_H

#include <cstdint>

#include <architecture/_GeneralModuleFiles/sys_model.h>
#include <architecture/messaging/messaging.h>
#include <architecture/msgPayloadDef/THRArrayCmdForceMsgPayload.h>
#include <architecture/msgPayloadDef/THRArrayConfigMsgPayload.h>
#include <architecture/msgPayloadDef/THRArrayOnTimeCmdMsgPayload.h>
#include <architecture/utilities/bskLogging.h>
#include <architecture/utilities/macroDefinitions.h>
#include "thrFiringSchmittAlgorithm.h"

class ThrFiringSchmitt : public SysModel {
   public:
    ThrFiringSchmitt();

    void reset(uint64_t callTime) override;
    void updateState(uint64_t callTime) override;
    double getLevelOn() const;
    void setLevelOn(double level);
    double getLevelOff() const;
    void setLevelOff(double level);
    double getThrMinFireTime() const;
    void setThrMinFireTime(double time);
    uint32_t getBaseThrustState() const;
    void setBaseThrustState(uint32_t state);

    /* declare module IO interfaces */
    ReadFunctor<THRArrayCmdForceMsgPayload> thrForceInMsg;  //!< The name of the Input message
    Message<THRArrayOnTimeCmdMsgPayload> onTimeOutMsg;      //!< The name of the output message*, onTimeOutMsg
    ReadFunctor<THRArrayConfigMsgPayload> thrConfInMsg;     //!< The name of the thruster cluster Input message

    BSKLogger bskLogger = {};  //!< BSK Logging

   private:
    ThrFiringSchmittAlgorithm algorithm;
};

#endif  // XMERA_THRFIRINGSCHMITT_H
