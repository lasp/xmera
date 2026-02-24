// SPDX-License-Identifier: ISC
// Copyright (c) 2016, Autonomous Vehicle System Lab, University of Colorado at Boulder
// Copyright (c) 2025, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

#ifndef _CMD_FORCE_INERTIAL_MESSAGE_
#define _CMD_FORCE_INERTIAL_MESSAGE_

/*! @brief Message used to define the vehicle control force vector in Inertial frame components*/
typedef struct {
    double forceRequestInertial[3];  //!< [N] control force request
} CmdForceInertialMsgPayload;

#endif
