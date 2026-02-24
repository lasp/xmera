// SPDX-License-Identifier: ISC
// Copyright (c) 2016, Autonomous Vehicle System Lab, University of Colorado at Boulder
// Copyright (c) 2025, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

#ifndef _RATE_STEERING_MSG_H
#define _RATE_STEERING_MSG_H

/*! @brief Structure used to define the output definition for attitude guidance*/
typedef struct {
    double omega_BastR_B[3];   //!< [r/s]   Desired body rate relative to R
    double omegap_BastR_B[3];  //!< [r/s^2] Body-frame derivative of omega_BastR_B
} RateCmdMsgPayload;

#endif
