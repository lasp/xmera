// SPDX-License-Identifier: ISC
// Copyright (c) 2016, Autonomous Vehicle System Lab, University of Colorado at Boulder
// Copyright (c) 2025, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

#ifndef INERTIAL_FILTER_MESSAGE_H
#define INERTIAL_FILTER_MESSAGE_H

#define AKF_N_STATES 6
#define MAX_N_ATT_STATES 4

/*! @brief structure for filter-states output for the unscented kalman filter
implementation of the inertial state estimator*/
typedef struct {
    double timeTag;                             //!< [s] Current time of validity for output
    double covar[AKF_N_STATES * AKF_N_STATES];  //!< [-] Current covariance of the filter
    double state[AKF_N_STATES];                 //!< [-] Current estimated state of the filter
    int numObs;                                 //!< [-] Valid observation count for this frame
} InertialFilterMsgPayload;

#endif
