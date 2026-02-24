// SPDX-License-Identifier: ISC
// Copyright (c) 2022, Autonomous Vehicle System Lab, University of Colorado at Boulder
// Copyright (c) 2025, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

#ifndef SIM_THRUSTER_BODYTOHUBINFO_H
#define SIM_THRUSTER_BODYTOHUBINFO_H
#include <stdint.h>

#include <Eigen/Dense>
/*! attached body to hub information structure*/
typedef struct
    //@cond DOXYGEN_IGNORE
    BodyToHubInfo
//@endcond
{
    Eigen::Vector3d r_FB_B;      //!< position vector of the frame F relative to frame B
    Eigen::Vector3d omega_FB_B;  //!< angular velocity of F relative to B
    Eigen::Matrix3d dcm_BF;      //!< DCM of B relative to F
} BodyToHubInfo;

#endif
