#ifndef XMERA_SIMPLESTORAGEUNIT_H
#define XMERA_SIMPLESTORAGEUNIT_H

#include <simulation/onboardDataHandling/_GeneralModuleFiles/dataStorageUnitBase.h>
#include <architecture/utilities/macroDefinitions.h>

/*! @brief simple storage unit class */
class SimpleStorageUnit : public DataStorageUnitBase {
   public:
    SimpleStorageUnit();
    ~SimpleStorageUnit();
    void setDataBuffer(int64_t data);  //!< Method to add/remove data from the storage unit once

   private:
    void customreset(uint64_t CurrentClock);       //!< Custom Reset method
    void integrateDataStatus(double currentTime);  //!< Overwrites the integrateDataStatus method to create a single
                                                   //!< partition in the storage unit ("STORED DATA")
};

#endif  // XMERA_SIMPLESTORAGEUNIT_H
