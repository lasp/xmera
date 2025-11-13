#ifndef MAPPINGINSTRUMENT_H
#define MAPPINGINSTRUMENT_H

#include <Eigen/Dense>
#include <string>
#include <vector>

#include <architecture/_GeneralModuleFiles/sys_model.h>
#include <architecture/messaging/messaging.h>
#include <architecture/msgPayloadDef/AccessMsgPayload.h>
#include <architecture/msgPayloadDef/DataNodeUsageMsgPayload.h>

/*! @brief This module receives a vector of accessMsgPayloads and outputs a vector of DataNodeUsageMsgPayloads for each
 * accessible point.
 */
class MappingInstrument : public SysModel {
   public:
    MappingInstrument();
    ~MappingInstrument();

    void reset(uint64_t currentSimNanos);
    void updateState(uint64_t currentSimNanos);
    void addMappingPoint(Message<AccessMsgPayload>* tmpAccessMsg,
                         std::string dataName);  //!< connects accessMsgPayload to instrument

   public:
    std::vector<Message<DataNodeUsageMsgPayload>*> dataNodeOutMsgs;  //!< vector of data node output messages
    std::vector<ReadFunctor<AccessMsgPayload>> accessInMsgs;         //!< vector of ground location access messages
    BSKLogger bskLogger;                                             //!< -- BSK Logging
    double nodeBaudRate = -1;                                        //!< [baud] Data provided (+).

   private:
    std::vector<std::string> mappingPoints;
    std::vector<DataNodeUsageMsgPayload> dataNodeOutMsgBuffer;  //!< buffer of data node output data
};

#endif  // MAPPINGINSTRUMENT_H
