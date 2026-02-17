// SPDX-License-Identifier: ISC
// Copyright (c) 2024, Autonomous Vehicle System Lab, University of Colorado at Boulder
// Copyright (c) 2025, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

#ifndef prescribedRotationSimMsg_h
#define prescribedRotationSimMsg_h

/*! @brief Structure used to define the prescribed motion state effector rotational state data message */
typedef struct {
    double omega_FM_F[3];       //!< [rad/s] Angular velocity of the F frame wrt the M frame in F frame components
    double omegaPrime_FM_F[3];  //!< [rad/s^2] B/M frame time derivative of omega_FM_F
    double sigma_FM[3];         //!< MRP attitude parameters for the F frame relative to the M frame
} PrescribedRotationMsgPayload;

#endif /* prescribedRotationSimMsg_h */
