// SPDX-License-Identifier: ISC
// Copyright (c) 2016, Autonomous Vehicle System Lab, University of Colorado at Boulder
// Copyright (c) 2025, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

#ifndef XMERA_DATANODEUSAGESIMMSG_H
#define XMERA_DATANODEUSAGESIMMSG_H

/*! @brief Message for reporting the science or telemetry data produced or consumed by a module.*/
typedef struct {
    // std::string dataName; //!< Data name
    char dataName[128];  //!< data name
    double
        baudRate;  //!< [bits/s] Data usage by the message writer; positive for data generators, negative for data sinks
} DataNodeUsageMsgPayload;

#endif  // XMERA_DATANODEUSAGESIMMSG_H
