// SPDX-License-Identifier: ISC
// Copyright (c) 2016, Autonomous Vehicle System Lab, University of Colorado at Boulder
// Copyright (c) 2025, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

#ifndef XMERA_SIMPLETRANSMITTER_H
#define XMERA_SIMPLETRANSMITTER_H

#include <simulation/onboardDataHandling/_GeneralModuleFiles/dataNodeBase.h>
#include <architecture/utilities/bskLogging.h>

/*! @brief simple data transmitter class */
class SimpleTransmitter : public DataNodeBase {
   public:
    SimpleTransmitter();
    ~SimpleTransmitter();
    void addStorageUnitToTransmitter(Message<DataStorageStatusMsgPayload>* tmpStorageUnitMsg);

   private:
    void evaluateDataModel(DataNodeUsageMsgPayload* dataUsageMsg, double currentTime);
    bool customReadMessages();
    int getMaxIndex();

   public:
    double packetSize;  //!< Size of packet to downklink (bytes)
    int numBuffers;     //!< Number of buffers the transmitter can access
    int currentIndex;   //!< Current partition that the transmitter is downlinking a packet for
    std::vector<ReadFunctor<DataStorageStatusMsgPayload>>
        storageUnitInMsgs;                                     //!< Vector of data node input message names
    std::vector<DataStorageStatusMsgPayload> storageUnitMsgs;  //!< local copies of input messages
    BSKLogger bskLogger;                                       //!< class variable

   private:
    double packetTransmitted;  //!< Amount of packet downlinked (bytes)
    double currentTimestep;    //!< Current timestep tracked for data packet integration
    double previousTime;       //!< Previous timestep tracked for data packet integration
};

#endif  // XMERA_SIMPLETRANSMITTER_H
