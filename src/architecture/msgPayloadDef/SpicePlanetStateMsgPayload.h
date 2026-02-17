// SPDX-License-Identifier: ISC
// Copyright (c) 2016, Autonomous Vehicle System Lab, University of Colorado at Boulder
// Copyright (c) 2025, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

#ifndef SpicePlanetState_MESSAGE_H
#define SpicePlanetState_MESSAGE_H

#define MAX_BODY_NAME_LENGTH 64

//! @brief The SPICE planet statate structure is the struct used to ouput planetary body states to the messaging system
typedef struct {
    double J2000Current;          //!< s Time of validity for the planet state
    double PositionVector[3];     //!< m True position of the planet for the time
    double VelocityVector[3];     //!< m/s True velocity of the planet for the time
    double J20002Pfix[3][3];      //!< (-) Orientation matrix of planet-fixed relative to inertial
    double J20002Pfix_dot[3][3];  //!< (-) Derivative of the orientation matrix of planet-fixed relative to inertial
    int computeOrient;            //!< (-) Flag indicating whether the reference should be computed
    char PlanetName[MAX_BODY_NAME_LENGTH];  //!< -- Name of the planet for the state
} SpicePlanetStateMsgPayload;

#endif
