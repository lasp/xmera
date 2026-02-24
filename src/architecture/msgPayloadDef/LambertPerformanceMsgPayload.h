// SPDX-License-Identifier: ISC
// Copyright (c) 2023, Autonomous Vehicle System Lab, University of Colorado at Boulder
// Copyright (c) 2025, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

#ifndef LAMBERT_PERFORMANCE_MESSAGE_H
#define LAMBERT_PERFORMANCE_MESSAGE_H

/*! @brief Structure used to define extra output of the Lambert problem solution */
typedef struct {
    double x;         //!< [-] solution for free variable (iteration variable)
    int numIter;      //!< [-] number of root-finder iterations to find x
    double errX;      //!< [-] difference in x between last and second-to-last iteration
    double xSol2;     //!< [-] second solution for free variable (iteration variable)
    int numIterSol2;  //!< [-] number of root-finder iterations to find x_sol2
    double errXSol2;  //!< [-] difference in x_sol2 between last and second-to-last iteration
} LambertPerformanceMsgPayload;

#endif
