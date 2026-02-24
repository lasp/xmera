// SPDX-License-Identifier: ISC
// Copyright (c) 2016, Autonomous Vehicle System Lab, University of Colorado at Boulder
// Copyright (c) 2025, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

#ifndef DIPOLE_REQUEST_BODY_MSG_H
#define DIPOLE_REQUEST_BODY_MSG_H

/*! @brief Structure used to define the output of the sub-module.  This is the same
    output message that is used by all sub-modules in the module folder. */
typedef struct {
    double dipole_B[3];  //!< [A-m2] desired net magnetic torque bar dipole in the Body frame.
} DipoleRequestBodyMsgPayload;

#endif
