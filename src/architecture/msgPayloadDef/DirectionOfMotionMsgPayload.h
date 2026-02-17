// SPDX-License-Identifier: ISC
// Copyright (c) 2015, Autonomous Vehicle System Lab, University of Colorado at Boulder
// Copyright (c) 2023, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

#ifndef DIRECTION_MOTION_MSG_H
#define DIRECTION_MOTION_MSG_H

//!@brief Optical Navigation measurement message containing the matched key points between two images
/*! This message is output by the optical flow module and contains the key points shared between the two image
 * that were processed, as well as the attitudes, validity, time tags, the camera ID, and the number of points detected.
 */
typedef struct
    //@cond DOXYGEN_IGNORE
    DirectionOfMotionMsgPayload
//@endcond
{
    bool valid;                        //!< --  Quality of measurement
    uint64_t cameraID;                 //!< --  Camera who's motion is in question
    uint64_t timeOfDirectionEstimate;  //!< --  time of the direction of motion computation
    double v_C_hat[3];                 //!< -- [-]   Camera direction of motion
    double covar_C[3 * 3];             //!< -- [-]   Unvertainty in direction of motion
} DirectionOfMotionMsgPayload;

#endif /* DIRECTION_MOTION_MSG_H */
