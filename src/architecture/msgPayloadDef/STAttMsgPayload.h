// SPDX-License-Identifier: ISC
// Copyright (c) 2016, Autonomous Vehicle System Lab, University of Colorado at Boulder
// Copyright (c) 2025, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

#ifndef ST_ATTITUDE_MESSAGE_H
#define ST_ATTITUDE_MESSAGE_H

/*! @brief Output structure for ST attitude measurement in vehicle body frame*/
typedef struct {
    double timeTag;          //!< [s] Vehicle time code associated with measurement
    double MRP_BdyInrtl[3];  //!< [-] MRP estimate of inertial to body transformation
    double omega_BN_B[3];    //!< [rad/s] Platform inertial angular velocity
    double dcm_CB[9];        //!< Star Tracker mount frame in the body frame
} STAttMsgPayload;

#endif
