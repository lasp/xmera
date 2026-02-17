// SPDX-License-Identifier: ISC
// Copyright (c) 2016, Autonomous Vehicle System Lab, University of Colorado at Boulder
// Copyright (c) 2025, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

#ifndef _FSW_DEFINITIONS_H
#define _FSW_DEFINITIONS_H

/* Boolean Definition */
typedef enum { BOOL_FALSE = 0, BOOL_TRUE } boolean_t;

/*! @brief Structure used to define the output definition for component availability */
typedef enum {
    AVAILABLE = 0, /* must be 0, so that if these states are set to zero, device is AVAILABLE be default */
    UNAVAILABLE
} FSWdeviceAvailability;

#endif
