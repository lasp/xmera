/*
 * ADCSDefinitions.h
 *
 * Provide generation defintions related to the ADCS Algorithms
 *
 * University of Colorado, Autonomous Vehicle Systems (AVS) Lab
 * Unpublished Copyright (c) 2012-2015 University of Colorado, All Rights Reserved
 */

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
