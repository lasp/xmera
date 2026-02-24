// SPDX-License-Identifier: ISC
// Copyright (c) 2016, Autonomous Vehicle System Lab, University of Colorado at Boulder
// Copyright (c) 2025, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

#ifndef OPNAV_MESSAGE_H
#define OPNAV_MESSAGE_H

/*! @brief structure for filter-states output for the unscented kalman filter
 implementation of the sunline state estimator*/
typedef struct {
    double timeTag;         //!< [s] Current time of validity for output
    int valid;              //!< Valid measurement if 1, invalid if 0
    double covar_N[3 * 3];  //!< [m^2] Current covariance of the filter
    double covar_B[3 * 3];  //!< [m^2] Current covariance of the filter
    double covar_C[3 * 3];  //!< [m^2] Current covariance of the filter
    double r_BN_N[3];       //!< [m] Current estimated state of the filter
    double r_BN_B[3];       //!< [m] Current estimated state of the filter
    double r_BN_C[3];       //!< [m] Current estimated state of the filter
    int planetID;           //!< [-] Planet being navigated, Earth=1, Mars=2, Jupiter=3
    int faultDetected;      //!< [-] Bool if a fault is detected
} OpNavMsgPayload;

#endif
