// SPDX-License-Identifier: ISC
// Copyright (c) 2023, Autonomous Vehicle System Lab, University of Colorado at Boulder
// Copyright (c) 2025, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

#ifndef LAMBERT_PROBLEM_MESSAGE_H
#define LAMBERT_PROBLEM_MESSAGE_H

typedef enum { IZZO, GOODING } SolverMethod;

/*! @brief Structure used to define the input for Lambert problem */
typedef struct {
    SolverMethod solverMethod;  //!< [-] lambert solver algorithm (GOODING or IZZO)
    double r1vec[3];            //!< [m] position vector at t0
    double r2vec[3];            //!< [m] position vector at t1
    double transferTime;        //!< [s] time of flight between r1vec and r2vec (t1-t0)
    double mu;                  //!< [m^3 s^-2] gravitational parameter of body
    int numRevolutions;         //!< [-] number of revolutions to be completed (completed orbits)
} LambertProblemMsgPayload;

#endif
