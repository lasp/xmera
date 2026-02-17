// SPDX-License-Identifier: ISC
// Copyright (c) 2016, Autonomous Vehicle System Lab, University of Colorado at Boulder
// Copyright (c) 2025, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

#ifndef ACCESSSIMMSG_H
#define ACCESSSIMMSG_H

#include <cstdint>

/*! @brief Message that defines access to spacecraft from a groundLocation, providing access, range, and elevation with
 * repect to a ground location.
 */
typedef struct {
    uint64_t hasAccess;  //!< [-] 1 when the writer has access to a spacecraft; 0 otherwise.
    double slantRange;   //!< [m] Range from a location to the spacecraft.
    double elevation;    //!< [rad] Elevation angle for a given spacecraft.
    double azimuth;      //!< [rad] Azimuth angle for a spacecraft.
    double range_dot;    //!< [m/s] Range rate of a given spacecraft relative to a location in the SEZ rotating frame.
    double el_dot;       //!< [rad/s] Elevation angle rate for a given spacecraft in the SEZ rotating frame.
    double az_dot;       //!< [rad/s] Azimuth angle rate for a given spacecraft in the SEZ rotating frame.
    double r_BL_L[3];    //!<[m] Spacecraft position relative to the groundLocation in the SEZ frame.
    double v_BL_L[3];    //!<[m/s] SEZ relative time derivative of r_BL vector in SEZ vector components.
} AccessMsgPayload;

#endif /* accessSimMsg.h */
