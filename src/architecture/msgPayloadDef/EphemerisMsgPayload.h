// SPDX-License-Identifier: ISC
// Copyright (c) 2016, Autonomous Vehicle System Lab, University of Colorado at Boulder
// Copyright (c) 2025, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

#ifndef EPHEMERIS_OUTPUT_H
#define EPHEMERIS_OUTPUT_H

/*! @brief Message structure used to write ephemeris states out to other modules*/
typedef struct {
    double r_BdyZero_N[3];  //!< [m] Position of orbital body
    double v_BdyZero_N[3];  //!< [m/s] Velocity of orbital body
    double sigma_BN[3];     //!< MRP attitude of the orbital body fixed frame relative to inertial
    double omega_BN_B[3];   //!< [r/s] angular velocity of the orbital body relative to inertial
    double timeTag;         //!< [s] vehicle Time-tag for state
} EphemerisMsgPayload;

#endif
