// SPDX-License-Identifier: ISC
// Copyright (c) 2023, Autonomous Vehicle System Lab, University of Colorado at Boulder
// Copyright (c) 2025, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

#ifndef CAMERA_LOCALIZATION_MSG_H
#define CAMERA_LOCALIZATION_MSG_H

/*! @brief This structure includes the estimated camera location obtained from triangulation given a point cloud*/
typedef struct {
    bool valid;                  //!< [-] Quality of measurement
    int64_t cameraID;            //!< [-] ID of the camera
    uint64_t timeTag;            //!< [ns] vehicle time-tag when images where taken
    double sigma_BN[3];          //!< [-] MRP orientation of spacecraft when images where taken
    double cameraPos_N[3];       //!< [m] Camera location in inertial frame */
    double covariance_N[3 * 3];  //!< [m^2] covariance of estimated camera location in inertial frame
} CameraLocalizationMsgPayload;

#endif
