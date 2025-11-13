#ifndef XMERA_POWERNODEBASE_H
#define XMERA_POWERNODEBASE_H

#include <architecture/_GeneralModuleFiles/sys_model.h>
#include <architecture/messaging/messaging.h>
#include <Eigen/Dense>
#include <string>
#include <vector>

#include <architecture/msgPayloadDef/DeviceStatusMsgPayload.h>
#include <architecture/msgPayloadDef/PowerNodeUsageMsgPayload.h>

#include <architecture/utilities/bskLogging.h>

/*! @brief power node base class */
class PowerNodeBase : public SysModel {
   public:
    PowerNodeBase();
    ~PowerNodeBase();
    void reset(uint64_t currentSimNanos);
    void computePowerStatus(double currentTime);
    void updateState(uint64_t currentSimNanos);

   protected:
    void writeMessages(uint64_t CurrentClock);
    bool readMessages();
    virtual void evaluatePowerModel(
        PowerNodeUsageMsgPayload*
            powerUsageMsg) = 0;  //!< Virtual void method used to compute module-wise power usage/generation.
    virtual void customreset(uint64_t CurrentClock);          //!< Custom Reset method, similar to customSelfInit.
    virtual void customWriteMessages(uint64_t CurrentClock);  //!< custom Write method, similar to customSelfInit.
    virtual bool customReadMessages();  //!< Custom read method, similar to customSelfInit; returns `true' by default.

   public:
    Message<PowerNodeUsageMsgPayload> nodePowerOutMsg;    //!< output power message
    ReadFunctor<DeviceStatusMsgPayload> nodeStatusInMsg;  //!< note status input message
    double nodePowerOut;                                  //!< [W] Power provided (+) or consumed (-).
    uint64_t
        powerStatus;  //!< Device power mode; by default, 0 is off and 1 is on. Additional modes can fill other slots
    BSKLogger bskLogger;  //!< -- BSK Logging

   protected:
    PowerNodeUsageMsgPayload nodePowerMsg;  //!< buffer of output message
    DeviceStatusMsgPayload nodeStatusMsg;   //!< copy of input message
    double currentPowerConsumption;         //!< class variable
    double previousTime;                    //!< Previous time used for integration
};

#endif  // XMERA_POWERNODEBASE_H
