// SPDX-License-Identifier: ISC
// Copyright (c) 2016, Autonomous Vehicle System Lab, University of Colorado at Boulder
// Copyright (c) 2025, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

#include <cstdint>

#include <architecture/utilities/bskLogging.h>
#include "partitionedStorageUnit.h"

/*! The constructor creates a partitionedStorageUnit instance with zero stored data
 @return void;
 */
PartitionedStorageUnit::PartitionedStorageUnit() {
    this->storageCapacity = 0;
    this->storedDataSum = 0;
    return;
}

/*! Destructor.
 @return void
 */
PartitionedStorageUnit::~PartitionedStorageUnit() { return; }

/*! Custom reset function.
 @param currentClock
 @return void
 */
void PartitionedStorageUnit::customreset(uint64_t currentClock) {
    if (this->storageCapacity <= 0) {
        bskLogger.bskLog(BSK_INFORMATION, "The storageCapacity variable must be set to a positive value.");
    }
    return;
}

/*! Adds a partition to the storageUnit
 @param dataName
 @return void
 */
void PartitionedStorageUnit::addPartition(std::string dataName) {
    dataInstance tmpDataInstance;
    strncpy(tmpDataInstance.dataInstanceName, dataName.c_str(), sizeof(tmpDataInstance.dataInstanceName) - 1);
    tmpDataInstance.dataInstanceSum = 0;
    this->storedData.push_back(tmpDataInstance);
    return;
}

/*! Adds a specific amount of data to the specified partitions once
 @param partitionNames  //Vector of partition names
 @param data            //Vector of data to be added to each partition in partitionNames
 @return void
 */
void PartitionedStorageUnit::setDataBuffer(std::vector<std::string> partitionNames, std::vector<long long int> data) {
    for (size_t i = 0; i < partitionNames.size(); i++) {
        PartitionedStorageUnit::DataStorageUnitBase::setDataBuffer(partitionNames[i], data[i]);
    }
}
