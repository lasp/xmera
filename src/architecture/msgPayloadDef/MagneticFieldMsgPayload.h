// SPDX-License-Identifier: ISC
// Copyright (c) 2016, Autonomous Vehicle System Lab, University of Colorado at Boulder
// Copyright (c) 2025, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

#ifndef magneticFieldSimMsg_H
#define magneticFieldSimMsg_H

/*! magnetic field message definition */
typedef struct {
    double magField_N[3];  //!< [Tesla] Local magnetic field
} MagneticFieldMsgPayload;
#endif
