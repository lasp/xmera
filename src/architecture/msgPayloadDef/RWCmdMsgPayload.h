// SPDX-License-Identifier: ISC
// Copyright (c) 2016, Autonomous Vehicle System Lab, University of Colorado at Boulder
// Copyright (c) 2025, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

#ifndef SIM_RW_CMD_H
#define SIM_RW_CMD_H

/*! @brief Structure used to define the individual RW motor torque command message*/
typedef struct {
    double u_cmd;  //!< [Nm], torque command for individual RW
} RWCmdMsgPayload;

#endif
