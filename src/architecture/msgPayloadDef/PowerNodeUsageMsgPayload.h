// SPDX-License-Identifier: ISC
// Copyright (c) 2016, Autonomous Vehicle System Lab, University of Colorado at Boulder
// Copyright (c) 2025, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

#ifndef XMERA_POWERNODEUSAGESIMMSG_H
#define XMERA_POWERNODEUSAGESIMMSG_H

/*! @brief Message for reporting the power consumed produced or consumed by a module.*/
typedef struct {
    double netPower;  //!< [W] Power usage by the message writer; positive for sources, negative for sinks
} PowerNodeUsageMsgPayload;
#endif  // XMERA_POWERNODEUSAGESIMMSG_H
