// SPDX-License-Identifier: ISC
// Copyright (c) 2016, Autonomous Vehicle System Lab, University of Colorado at Boulder
// Copyright (c) 2025, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

#ifndef ATT_STATE_MESSAGE_H
#define ATT_STATE_MESSAGE_H

/*! @brief Structure used to define the output euler set for attitude reference generation */
typedef struct {
    double
        state[3];    //!< []   3D attitude orientation coordinate set The units depend on the attitude coordinate chosen
                     //!< and can be either radians (i.e. Euler angles) or dimensionless (i.e. MRP, quaternions, etc.)
    double rate[3];  //!< []   3D attitude rate coordinate set.  These rate coordinates can be either omega (in rad/sec)
                     //!< or attitude coordiante rates with appropriate units
} AttStateMsgPayload;

#endif
