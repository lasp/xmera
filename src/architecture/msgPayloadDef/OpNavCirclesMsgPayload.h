// SPDX-License-Identifier: ISC
// Copyright (c) 2025, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

#ifndef CIRCLE_OPNAV_MSG_H
#define CIRCLE_OPNAV_MSG_H

/*! @brief Structure used to define circles processed from image*/

#define MAX_CIRCLE_NUM 10

typedef struct {
    uint64_t timeTag;                           //!< --[ns]   Current vehicle time-tag associated with measurements
    int valid;                                  //!< --  Valid measurement if 1, not if 0
    int64_t cameraID;                           //!< -- [-]   ID of the camera that took the snapshot
    double planetIds[MAX_CIRCLE_NUM];           //!< -- [-]   Ids for identified celestial bodies
    double circlesCenters[2 * MAX_CIRCLE_NUM];  //!< -- [-]   Center x, y in pixels of the circles
    double circlesRadii[MAX_CIRCLE_NUM];        //!< -- [-]   Radius rho in pixels of the circles
    double uncertainty[3 * 3];  //!< -- [-] Uncertainty about the image processing results for x, y, rho (center and
                                //!< radius) for main circle
} OpNavCirclesMsgPayload;

#endif
