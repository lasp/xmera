#include <architecture/_GeneralModuleFiles/sys_model.h>
#include <Eigen/Dense>
#include <string>
#include <vector>

#include <architecture/messaging/messaging.h>
#include <architecture/msgPayloadDef/PowerNodeUsageMsgPayload.h>
#include <architecture/msgPayloadDef/PowerStorageStatusMsgPayload.h>

#include <architecture/utilities/bskLogging.h>

#ifndef XMERA_SIMPOWERSTORAGEBASE_H
#define XMERA_SIMPOWERSTORAGEBASE_H

/*! @brief power storage base class */
class PowerStorageBase : public SysModel {
   public:
    PowerStorageBase();
    ~PowerStorageBase();
    void reset(uint64_t currentSimNanos);
    void addPowerNodeToModel(Message<PowerNodeUsageMsgPayload>* tmpNodeMsg);
    void updateState(uint64_t currentSimNanos);

   protected:
    void writeMessages(uint64_t CurrentClock);
    bool readMessages();
    void integratePowerStatus(
        double currentTime);  //!< Integrates the net power given the current time using a simple Euler method.
    double sumAllInputs();    //!< Sums over the input power consumption messages.
    virtual void evaluateBatteryModel(
        PowerStorageStatusMsgPayload* msg) = 0;  //!< Virtual function to represent power storage computation or losses.
    virtual void customreset(uint64_t CurrentClock);             //!< Custom reset() method, similar to customSelfInit.
    virtual void customWriteMessages(uint64_t currentSimNanos);  //!< Custom Write() method, similar to customSelfInit.
    virtual bool customReadMessages();                           //!< Custom Read() method, similar to customSelfInit.

   public:
    std::vector<ReadFunctor<PowerNodeUsageMsgPayload>>
        nodePowerUseInMsgs;                                //!< Vector of power node input message names
    Message<PowerStorageStatusMsgPayload> batPowerOutMsg;  //!< power storage status output message
    double storedCharge_Init;  //!< [W-s] Initial stored charge set by the user. Defaults to 0.
    BSKLogger bskLogger;       //!< -- BSK Logging

   protected:
    PowerStorageStatusMsgPayload storageStatusMsg;       //!< class variable
    std::vector<PowerNodeUsageMsgPayload> nodeWattMsgs;  //!< class variable
    double previousTime;                                 //!< Previous time used for integration
    double currentTimestep;                              //!< [s] Timestep duration in seconds.
    double storedCharge;                                 //!< [W-s] Stored charge in Watt-hours.
    double currentPowerSum;                              //!< [W] Current net power.
};

#endif  // XMERA_SIMPOWERSTORAGEBASE_H
