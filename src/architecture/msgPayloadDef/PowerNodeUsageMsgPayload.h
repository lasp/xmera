#ifndef XMERA_POWERNODEUSAGESIMMSG_H
#define XMERA_POWERNODEUSAGESIMMSG_H

/*! @brief Message for reporting the power consumed produced or consumed by a module.*/
typedef struct {
    double netPower;  //!< [W] Power usage by the message writer; positive for sources, negative for sinks
} PowerNodeUsageMsgPayload;
#endif  // XMERA_POWERNODEUSAGESIMMSG_H
