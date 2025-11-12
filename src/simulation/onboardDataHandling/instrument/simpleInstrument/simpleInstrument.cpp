#include "simpleInstrument.h"

/*! Constructor, which sets the default nodeDataOut to zero.
 @return void
*/
SimpleInstrument::SimpleInstrument(){
    this->nodeBaudRate = 0.0;
    return;
}

/*! Destructor.
 @return void
 */
SimpleInstrument::~SimpleInstrument(){
    return;
}

/*! Sets the name and baud rate for the data in the output message.
 @return void
*/
void SimpleInstrument::evaluateDataModel(DataNodeUsageMsgPayload *dataUsageSimMsg, double currentTime){
    dataUsageSimMsg->baudRate = this->nodeBaudRate;
    strncpy (dataUsageSimMsg->dataName, this->nodeDataName, sizeof(dataUsageSimMsg->dataName));
    return;
}
