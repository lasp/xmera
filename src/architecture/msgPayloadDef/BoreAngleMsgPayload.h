// SPDX-License-Identifier: ISC
// Copyright (c) 2016, Autonomous Vehicle System Lab, University of Colorado at Boulder
// Copyright (c) 2025, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

#ifndef SIM_BORE_ANGLE_H
#define SIM_BORE_ANGLE_H

/*! @brief Structure used to compute a angle between boresight and body */
typedef struct {
    double azimuth;    //!< [r] the location angle to put the miss in a quadrant
    double missAngle;  //!< [r] the angular distance between the boresight and body
} BoreAngleMsgPayload;

#endif
