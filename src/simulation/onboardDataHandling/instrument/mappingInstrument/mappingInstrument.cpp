#include "mappingInstrument.h"
#include <string.h>

/*! This is the constructor for the module class.  It sets default variable
    values and initializes the various parts of the model */
MappingInstrument::MappingInstrument() {}

/*! Module Destructor */
MappingInstrument::~MappingInstrument() {
    for (long unsigned int c = 0; c < this->dataNodeOutMsgs.size(); c++) {
        delete this->dataNodeOutMsgs.at(c);
    }
}

/*! This method is used to reset the module. The nodeBaudRate is checked for a non-zero value.
 @param currentSimNanos
 @return void
 */
void MappingInstrument::reset(uint64_t currentSimNanos) {
    // check that the baud rate is set
    if (this->nodeBaudRate < 0.0) {
        bskLogger.bskLog(BSK_ERROR, "MappingInstrument.nodeBaudRate is not set to a positive value.");
    }

    return;
}

/*! This method updates the state by reading messages, calling computeDataStatus, and writing messages
 @param currentSimNanos
 @return void
 */
void MappingInstrument::updateState(uint64_t currentSimNanos) {
    /* Loop through each access message */
    for (long unsigned int c = 0; c < this->accessInMsgs.size(); c++) {
        /* Zero the output message buffer */
        this->dataNodeOutMsgBuffer.at(c) = DataNodeUsageMsgPayload{};

        /* Read the access message */
        AccessMsgPayload accessMsg{};
        accessMsg = this->accessInMsgs.at(c)();

        /* Check for access, set the data rate */
        if (accessMsg.hasAccess) {
            dataNodeOutMsgBuffer.at(c).baudRate = this->nodeBaudRate;
        } else {
            dataNodeOutMsgBuffer.at(c).baudRate = 0;
        }

        strcpy(dataNodeOutMsgBuffer.at(c).dataName, mappingPoints[c].c_str());

        /* Write the output message */
        this->dataNodeOutMsgs.at(c)->write(&this->dataNodeOutMsgBuffer.at(c), this->moduleID, currentSimNanos);
    }

    return;
}

/*! Adds a mapping point (access message and name) to the module
 * @return void
 */
void MappingInstrument::addMappingPoint(Message<AccessMsgPayload>* tmpAccessMsg, std::string dataName) {
    /* Add the name of the mapping point */
    this->mappingPoints.push_back(dataName);

    /* Add the access message */
    this->accessInMsgs.push_back(tmpAccessMsg->addSubscriber());

    /* Create buffer output messages */
    Message<DataNodeUsageMsgPayload>* msg;
    msg = new Message<DataNodeUsageMsgPayload>;
    this->dataNodeOutMsgs.push_back(msg);

    /* Expand the data node usage buffer vectors */
    this->dataNodeOutMsgBuffer.emplace_back();

    return;
}
