// SPDX-License-Identifier: ISC
// Copyright (c) 2024, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

#ifndef COMOPNAVMSG_H
#define COMOPNAVMSG_H

//!@brief Center of mass optical navigation measurement message
/*! This message is output by the center of brightness converter module and contains the center of mass (COM) after
 * offsetting the center of brightness (COB) using the phase angle correction.
 */
typedef struct
    //@cond DOXYGEN_IGNORE
    OpNavCOMMsgPayload
//@endcond
{
    uint64_t timeTag;              //!< --[ns] Current vehicle time-tag associated with measurements
    bool valid;                    //!< -- Quality of measurement
    int64_t cameraID;              //!< -- [-] ID of the camera that took the image
    double centerOfMass[2];        //!< -- [-] Center x, y of bright pixels after correction
    double centerOfBrightness[2];  //!< -- [-] Center x, y of bright pixels
    double offsetFactor;           //!< -- [-] COM/COB offset factor as a fraction of object radius
    int32_t objectPixelRadius;     //!< -- [-] radius of object in pixels
    double phaseAngle;             //!< -- [rad] angle between Sun-Object-Camera
    double sunDirection;           //!< -- [rad] Sun direction in the image
} OpNavCOMMsgPayload;

#endif /* COMOPNAVMSG_H */
