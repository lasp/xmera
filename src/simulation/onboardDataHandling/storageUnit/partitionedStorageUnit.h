// SPDX-License-Identifier: ISC
// Copyright (c) 2016, Autonomous Vehicle System Lab, University of Colorado at Boulder
// Copyright (c) 2025, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

#ifndef XMERA_PARTITIONEDSTORAGEUNIT_H
#define XMERA_PARTITIONEDSTORAGEUNIT_H

#include <simulation/onboardDataHandling/_GeneralModuleFiles/dataStorageUnitBase.h>
#include <architecture/utilities/macroDefinitions.h>

/*! @brief partioned storage unit class */
class PartitionedStorageUnit : public DataStorageUnitBase {
   public:
    PartitionedStorageUnit();
    ~PartitionedStorageUnit();
    void addPartition(std::string dataName);
    void setDataBuffer(std::vector<std::string> partitionNames,
                       std::vector<long long int> data);  //!< Adds/removes the data from the partitionNames partitions

   private:
    void customreset(uint64_t CurrentClock) override;
};

#endif  // XMERA_PARTITIONEDSTORAGEUNIT_H
