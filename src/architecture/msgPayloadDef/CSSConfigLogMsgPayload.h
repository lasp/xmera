// SPDX-License-Identifier: ISC
// Copyright (c) 2016, Autonomous Vehicle System Lab, University of Colorado at Boulder
// Copyright (c) 2025, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

#ifndef CSSCONFIGLOGSIMMSG_H
#define CSSCONFIGLOGSIMMSG_H

//!@brief CSS configuration message log message
/*! This message is the outpout of each CSS device to log all the configuration and
    measurement states.
 */
typedef struct
    //@cond DOXYGEN_IGNORE
    CSSConfigLogMsgPayload
//@endcond
{
    double r_B[3] = {0};  //!< [m] sensor position vector in the spacecraft, "B", body frame
    double nHat_B[3];     //!< [] sensor unit direction vector in the spacecraft, "B", body frame
    double fov;           //!< [rad] field of view (boresight to edge)
    double signal;        //!< [] current sensor signal
    double maxSignal;     //!< [] maximum sensor signal, -1 means this value is not set
    double minSignal;     //!< [] maximum sensor signal, -1 means this value is not set
    int CSSGroupID = 0;   //!< [] Group ID if the CSS is part of a cluster
} CSSConfigLogMsgPayload;

#endif /* CSSCONFIGLOGSIMMSG_H */
