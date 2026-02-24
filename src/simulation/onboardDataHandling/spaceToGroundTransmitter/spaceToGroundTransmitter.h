// SPDX-License-Identifier: ISC
// Copyright (c) 2016, Autonomous Vehicle System Lab, University of Colorado at Boulder
// Copyright (c) 2025, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

#ifndef XMERA_SPACETOGROUNDTRANSMITTER_H
#define XMERA_SPACETOGROUNDTRANSMITTER_H

#include <architecture/msgPayloadDef/AccessMsgPayload.h>
#include <simulation/onboardDataHandling/_GeneralModuleFiles/dataNodeBase.h>

#include <architecture/utilities/bskLogging.h>

/*! @brief space to ground data transmitter class */
class SpaceToGroundTransmitter : public DataNodeBase {
   public:
    SpaceToGroundTransmitter();
    ~SpaceToGroundTransmitter();
    void addStorageUnitToTransmitter(Message<DataStorageStatusMsgPayload>* tmpStorageUnitMsg);
    void addAccessMsgToTransmitter(Message<AccessMsgPayload>* tmpAccessMsg);

   private:
    void evaluateDataModel(DataNodeUsageMsgPayload* dataUsageMsg, double currentTime);
    bool customReadMessages();

   public:
    double packetSize;  //!< Size of packet to downklink (bytes)
    int numBuffers;     //!< Number of buffers the transmitter can access
    std::vector<ReadFunctor<DataStorageStatusMsgPayload>>
        storageUnitInMsgs;  //!< vector of input messages for storage unit messages
    std::vector<ReadFunctor<AccessMsgPayload>>
        groundLocationAccessInMsgs;  //!< vector of input message for ground location access
    std::vector<DataStorageStatusMsgPayload> storageUnitMsgsBuffer;  //!< local copy of data storage messages
    uint64_t hasAccess;                                              //!< class variable
    BSKLogger bskLogger;                                             //!< class variable

   private:
    double packetTransmitted;                                //!< Amount of packet downlinked (bytes)
    double currentTimestep;                                  //!< Current timestep tracked for data packet integration
    double previousTime;                                     //!< Previous timestep tracked for data packet integration
    std::vector<AccessMsgPayload> groundLocationAccessMsgs;  //!< local copy of ground access messages
};
#endif  // XMERA_SPACETOGROUNDTRANSMITTER_H
