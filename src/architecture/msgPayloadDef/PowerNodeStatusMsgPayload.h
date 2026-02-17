// SPDX-License-Identifier: ISC
// Copyright (c) 2016, Autonomous Vehicle System Lab, University of Colorado at Boulder
// Copyright (c) 2025, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

#ifndef XMERA_POWERNODESTATUSMSG_H
#define XMERA_POWERNODESTATUSMSG_H

//! @brief Power node command message used to change the state of power modules.
typedef struct {
    uint64_t powerStatus;  //!< Power status indicator; 0 is off, 1 is on, additional values
} PowerNodeStatusMsgPayload;

#endif  // XMERA_POWERNODESTATUSMSG_H
