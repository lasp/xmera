// SPDX-License-Identifier: ISC
// Copyright (c) 2016, Autonomous Vehicle System Lab, University of Colorado at Boulder
// Copyright (c) 2025, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

#ifndef OPNAVUVECMESSAGE_H
#define OPNAVUVECMESSAGE_H

//!@brief Optical navigation message for unit vector measurements
/*! This message contains the output of any measurement method (primarily image processing)
 * that outputs a unit vector from the target to the camera.
 */
typedef struct
    //@cond DOXYGEN_IGNORE
    OpNavUnitVecMsgPayload
//@endcond
{
    double timeTag;         //!< [s] Current time of validity for output
    bool valid;             //!< Quality of measurement if 1, invalid if 0
    double covar_N[3 * 3];  //!< [-] Covariance of measurement in the inertial frame
    double covar_B[3 * 3];  //!< [-] Covariance of measurement in the body frame
    double covar_C[3 * 3];  //!< [-] Covariance of measurement in the camera frame
    double rhat_BN_N[3];    //!< [-] measurement in the inertial frame
    double rhat_BN_B[3];    //!< [-] measurement in the body frame
    double rhat_BN_C[3];    //!< [-] measurement in the camera frame
} OpNavUnitVecMsgPayload;

#endif /* OPNAVUVECMESSAGE_H */
