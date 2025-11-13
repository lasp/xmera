#include <cstdint>

#include "simpleStorageUnit.h"
#include <architecture/utilities/bskLogging.h>

/*! The constructor creates a SimpleStorageUnit instance with zero stored data
 @return void
 */
SimpleStorageUnit::SimpleStorageUnit() {
    this->storageCapacity = 0;
    this->storedDataSum = 0;
    return;
}

/*! Destructor
 @return void
 */
SimpleStorageUnit::~SimpleStorageUnit() { return; }

/*! Custom reset function
 @param currentClock
 */
void SimpleStorageUnit::customreset(uint64_t currentClock) {
    if (this->storageCapacity <= 0) {
        bskLogger.bskLog(BSK_INFORMATION, "The storageCapacity variable must be set to a positive value.");
    }
    return;
}

/*! Overwrites the integrateDataStatus method to create a single partition in the storage unit ("STORED DATA")
 @param currentTime
 @return void
 */
void SimpleStorageUnit::integrateDataStatus(double currentTime) {
    this->currentTimestep = currentTime - this->previousTime;
    this->netBaud = 0;

    //! - loop over all the data nodes and add them to the single partition.
    std::vector<DataNodeUsageMsgPayload>::iterator it;
    for (it = nodeBaudMsgs.begin(); it != nodeBaudMsgs.end(); it++) {
        if (storedData.size() == 0) {
            this->storedData.push_back({{'S', 'T', 'O', 'R', 'E', 'D', ' ', 'D', 'A', 'T', 'A'}, 0});
        } else if ((this->storedDataSum + round(it->baudRate * this->currentTimestep) < this->storageCapacity) ||
                   (it->baudRate <= 0)) {
            //! - Only perform the operation if it will not result in less than 0 data
            if ((this->storedData[0].dataInstanceSum + it->baudRate * this->currentTimestep) >= 0) {
                this->storedData[0].dataInstanceSum += round(it->baudRate * this->currentTimestep);
            }
        }
        this->netBaud += it->baudRate;
    }

    //!- Sum all data in storedData vector
    this->storedDataSum = this->storedData[0].dataInstanceSum;

    //!- Update previousTime
    this->previousTime = currentTime;
    return;
}

/*! Adds a specific amount of data to the storedData vector once
 @param data //Data to be added to the "STORED DATA" partition
 @return void
 */
void SimpleStorageUnit::setDataBuffer(int64_t data) {
    std::string partitionName = "STORED DATA";
    SimpleStorageUnit::DataStorageUnitBase::setDataBuffer(partitionName, data);
}
