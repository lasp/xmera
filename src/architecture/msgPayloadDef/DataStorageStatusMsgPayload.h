#include <string>
#include <vector>

#ifndef XMERA_DATASTORAGESTATUSSIMMSG_H
#define XMERA_DATASTORAGESTATUSSIMMSG_H

/*! @brief Message to store current storage unit stored data, storage capacity, and received data.*/
typedef struct
    //@cond DOXYGEN_IGNORE
    DataStorageStatusMsgPayload
//@endcond
{
    double storageLevel;     //!< [b] Storage unit stored data in bits.
    double storageCapacity;  //!< [b] Maximum data storage unit capacity.
    double currentNetBaud;   //!< [baud] Current data written to or removed from the storage unit net power.
    std::vector<std::string> storedDataName;  //!< [] vector of data name strings
    std::vector<double> storedData;           //!< [] vector of stored data amount for each data name group
} DataStorageStatusMsgPayload;

#endif  // XMERA_DATASTORAGESTATUSSIMMSG_H
