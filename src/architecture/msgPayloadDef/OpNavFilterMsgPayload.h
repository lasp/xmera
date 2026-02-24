// SPDX-License-Identifier: ISC
// Copyright (c) 2016, Autonomous Vehicle System Lab, University of Colorado at Boulder
// Copyright (c) 2025, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

#ifndef OPNAV_FILTER_MESSAGE_H
#define OPNAV_FILTER_MESSAGE_H

#define ODUKF_N_STATES 6
#define ODUKF_N_MEAS 3

/*! @brief structure for filter-states output for the unscented kalman filter
 implementation of the sunline state estimator*/
typedef struct {
    double timeTag;                                 //!< [s] Current time of validity for output
    double covar[ODUKF_N_STATES * ODUKF_N_STATES];  //!< [-] Current covariance of the filter
    double state[ODUKF_N_STATES];                   //!< [-] Current estimated state of the filter
    double stateError[ODUKF_N_STATES];              //!< [-] Current deviation of the state from the reference state
    double postFitRes[ODUKF_N_MEAS];                //!< [-] PostFit Residuals
    int numObs;                                     //!< [-] Valid observation count for this frame
} OpNavFilterMsgPayload;

#endif
