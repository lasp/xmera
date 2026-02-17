// SPDX-License-Identifier: ISC
// Copyright (c) 2021, Autonomous Vehicle System Lab, University of Colorado at Boulder
// Copyright (c) 2025, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

#ifndef MTB_MSG_H
#define MTB_MSG_H

/*! gravity gradient torque message definition */
typedef struct {
    double mtbNetTorque_B[3];  //!< [Nm]  net torque contribution of all magnetic torque bars in Body frame components
} MTBMsgPayload;

#endif  // MTB_MSG_H
