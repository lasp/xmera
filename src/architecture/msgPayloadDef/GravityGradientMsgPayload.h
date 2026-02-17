// SPDX-License-Identifier: ISC
// Copyright (c) 2016, Autonomous Vehicle System Lab, University of Colorado at Boulder
// Copyright (c) 2025, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

#ifndef gravityGradientSimMsg_H
#define gravityGradientSimMsg_H

/*! gravity gradient torque message definition */
typedef struct {
    double gravityGradientTorque_B[3];  //!< [Nm] Gravity Gradient torque in body frame components
} GravityGradientMsgPayload;

#endif  // gravityGradientSimMsg_H
