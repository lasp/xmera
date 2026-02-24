// SPDX-License-Identifier: ISC
// Copyright (c) 2016, Autonomous Vehicle System Lab, University of Colorado at Boulder
// Copyright (c) 2025, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

#ifndef COBOPNAVMSG_H
#define COBOPNAVMSG_H

#include <array>

//!@brief Center of brightness optical navigation measurement message
/*! This message is output by the center of brightness module and contains the center of brightness of the image
 * that was input, as well as the validity of the image processing process, the camera ID, and the number of pixels
 * found during the computation.
 */
typedef struct
    //@cond DOXYGEN_IGNORE
    OpNavCOBMsgPayload
//@endcond
{
    uint64_t timeTag;                 //!< --[ns]   Current vehicle time-tag associated with measurements
    bool valid;                       //!< --  Quality of measurement
    int64_t cameraID;                 //!< -- [-]   ID of the camera that took the image
    double centerOfBrightness[2];     //!< -- [-]   Center x, y of bright pixels
    int32_t pixelsFound;              //!< -- [-] Number of bright pixels found in the image
    double rollingAverageBrightness;  //!< [-] brightness computed over rolling average
} OpNavCOBMsgPayload;

#endif /* COBOPNAVMSG_H */
