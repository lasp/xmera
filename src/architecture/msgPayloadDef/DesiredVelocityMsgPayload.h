// SPDX-License-Identifier: ISC
// Copyright (c) 2023, Autonomous Vehicle System Lab, University of Colorado at Boulder
// Copyright (c) 2025, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

#ifndef DESIRED_VELOCITY_MESSAGE_H
#define DESIRED_VELOCITY_MESSAGE_H

/*! @brief Structure used to define a desired velocity for a maneuver in the inertial frame */
typedef struct {
    double vDesired_N[3];  //!< [m/s] Desired velocity in inertial frame N
    double maneuverTime;   //!< [s] Time of maneuver
} DesiredVelocityMsgPayload;

#endif
