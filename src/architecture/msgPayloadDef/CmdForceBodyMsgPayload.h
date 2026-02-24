// SPDX-License-Identifier: ISC
// Copyright (c) 2016, Autonomous Vehicle System Lab, University of Colorado at Boulder
// Copyright (c) 2025, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

#ifndef _CMD_FORCE_BODY_MESSAGE_
#define _CMD_FORCE_BODY_MESSAGE_

/*! @brief Message used to define the vehicle control force vector in Body frame components*/
typedef struct {
    double forceRequestBody[3];  //!< [N] Control force requested
} CmdForceBodyMsgPayload;

#endif
