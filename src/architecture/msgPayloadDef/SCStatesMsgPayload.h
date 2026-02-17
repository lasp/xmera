// SPDX-License-Identifier: ISC
// Copyright (c) 2016, Autonomous Vehicle System Lab, University of Colorado at Boulder
// Copyright (c) 2025, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

#ifndef SC_STATE_MESSAGE_H
#define SC_STATE_MESSAGE_H

#include <cstdint>

/*! @brief This structure is used in the messaging system to communicate what the
 state of the vehicle is currently.*/
typedef struct {
    double r_BN_N[3];                      //!< m  Current position vector (inertial)
    double v_BN_N[3];                      //!< m/s Current velocity vector (inertial)
    double r_CN_N[3];                      //!< m  Current position of CoM vector (inertial)
    double v_CN_N[3];                      //!< m/s Current velocity of CoM vector (inertial)
    double sigma_BN[3];                    //!< -- Current MRPs (inertial)
    double omega_BN_B[3];                  //!< r/s Current angular velocity
    double omegaDot_BN_B[3];               //!< r/s/s Current angular acceleration
    double TotalAccumDVBdy[3];             //!< m/s Accumulated DV of center of mass in body frame coordinates
    double TotalAccumDV_BN_B[3];           //!< m/s Accumulated DV of body frame in body frame coordinates
    double TotalAccumDV_CN_N[3];           //!< m/s Accumulated DV of center of mass in inertial frame coordinates
    double nonConservativeAccelpntB_B[3];  //!< m/s/s Current Spacecraft non-conservative body frame accel
    uint64_t MRPSwitchCount;               //!< -- Number of times that MRPs have switched
} SCStatesMsgPayload;

#endif
