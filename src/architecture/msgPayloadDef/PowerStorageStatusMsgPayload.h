#ifndef XMERA_POWERSTORAGESTATUSSIMMSG_H
#define XMERA_POWERSTORAGESTATUSSIMMSG_H

/*! @brief Message to store current battery stored charge, maximum charge, and received power.*/
typedef struct {
    double storageLevel;     //!< [W-s] Battery stored charge in Watt-hours.
    double storageCapacity;  //!< [W-s] Maximum battery storage capacity.
    double currentNetPower;  //!< [W] Current net power received/drained from the battery.
} PowerStorageStatusMsgPayload;

#endif  // XMERA_POWERSTORAGESTATUSSIMMSG_H
