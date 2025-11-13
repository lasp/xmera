#ifndef XMERA_DATANODEBASE_H
#define XMERA_DATANODEBASE_H

#include <architecture/_GeneralModuleFiles/sys_model.h>
#include <Eigen/Dense>
#include <string>
#include <vector>

#include <architecture/messaging/messaging.h>
#include <architecture/msgPayloadDef/DataNodeUsageMsgPayload.h>
#include <architecture/msgPayloadDef/DataStorageStatusMsgPayload.h>
#include <architecture/msgPayloadDef/DeviceCmdMsgPayload.h>

/*! @brief data node base class */
class DataNodeBase : public SysModel {
   public:
    DataNodeBase();
    ~DataNodeBase();
    void reset(uint64_t currentSimNanos);
    void computeDataStatus(double currentTime);
    void updateState(uint64_t currentSimNanos);

   protected:
    void writeMessages(uint64_t CurrentClock);
    bool readMessages();
    virtual void evaluateDataModel(
        DataNodeUsageMsgPayload* dataUsageMsg,
        double currentTime) = 0;  //!< Virtual void method used to compute module-wise data usage/generation.
    virtual void customreset(uint64_t CurrentClock);          //!< Custom Reset method, similar to customSelfInit.
    virtual void customWriteMessages(uint64_t CurrentClock);  //!< custom Write method, similar to customSelfInit.
    virtual bool customReadMessages();  //!< Custom read method, similar to customSelfInit; returns `true' by default.

   public:
    Message<DataNodeUsageMsgPayload> nodeDataOutMsg;   //!< Message name for the node's output message
    ReadFunctor<DeviceCmdMsgPayload> nodeStatusInMsg;  //!< String for the message name that tells the node it's status
    double nodeBaudRate;                               //!< [baud] Data provided (+) or consumed (-).
    char nodeDataName[128];                            //!< Name of the data node consuming or generating data.
    uint64_t dataStatus;  //!< Device data mode; by default, 0 is off and 1 is on. Additional modes can fill other slots

   protected:
    DataNodeUsageMsgPayload nodeDataMsg;  //!< class variable
    DeviceCmdMsgPayload nodeStatusMsg;    //!< class variable
};

#endif  // XMERA_DATANODEBASE_H
