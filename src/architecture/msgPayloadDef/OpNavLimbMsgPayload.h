// SPDX-License-Identifier: ISC
// Copyright (c) 2025, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

#ifndef LIMB_MSG_H
#define LIMB_MSG_H

#define MAX_LIMB_PNTS 2000

/*! @brief Structure used to define the message containing planet limb data for opNav*/
typedef struct {
    double timeTag;                        //!< --[s]   Current vehicle time-tag associated with measurements
    int valid;                             //!< --  Valid measurement if 1, not if 0
    int32_t numLimbPoints;                 //!< -- [-] Number of limb points found
    int64_t cameraID;                      //!< -- [-]   ID of the camera that took the snapshot
    double planetIds;                      //!< -- [-]   ID for identified celestial body
    double limbPoints[2 * MAX_LIMB_PNTS];  //!< -- [-] (x, y) in pixels of the limb points
} OpNavLimbMsgPayload;

#endif
