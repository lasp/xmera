// SPDX-License-Identifier: ISC
// Copyright (c) 2016, Autonomous Vehicle System Lab, University of Colorado at Boulder
// Copyright (c) 2025, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

#ifndef REAL_TIME_FACTOR_H
#define REAL_TIME_FACTOR_H

//! @brief Container for sim real time speed up/down factor.
typedef struct {
    double speedFactor;  //!< -- factor of real time at which the sim should run
} RealTimeFactorMsgPayload;

#endif
