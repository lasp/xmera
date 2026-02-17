// SPDX-License-Identifier: ISC
// Copyright (c) 2016, Autonomous Vehicle System Lab, University of Colorado at Boulder
// Copyright (c) 2025, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

#ifndef VEHICLE_CONFIG_MESSAGE_H
#define VEHICLE_CONFIG_MESSAGE_H

#include <stdint.h>

/*! @brief Structure used to define a common structure for top level vehicle information*/
typedef struct {
    double ISCPntB_B[9];        //!< [kg m^2] Spacecraft Inertia
    double CoM_B[3];            //!< [m] Center of mass of spacecraft in body
    double massSC;              //!< [kg] Spacecraft mass
    uint32_t CurrentADCSState;  //!< [-] Current ADCS state for subsystem
} VehicleConfigMsgPayload;

#endif
