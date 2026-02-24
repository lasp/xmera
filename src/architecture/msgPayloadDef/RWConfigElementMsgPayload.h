// SPDX-License-Identifier: ISC
// Copyright (c) 2016, Autonomous Vehicle System Lab, University of Colorado at Boulder
// Copyright (c) 2025, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

#ifndef _RW_CONFIG_ELEMENT_MESSAGE_H
#define _RW_CONFIG_ELEMENT_MESSAGE_H

/*! @brief Message used to define a single FSW RW configuration message */
typedef struct {
    double gsHat_B[3];  //!< [-] Spin axis unit vector of the wheel in structure
    double Js;          //!< [kgm2] Spin axis inertia of the wheel
    double uMax;        //!< [Nm]   maximum RW motor torque
} RWConfigElementMsgPayload;

#endif
