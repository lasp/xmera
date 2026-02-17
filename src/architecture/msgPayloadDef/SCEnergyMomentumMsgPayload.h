// SPDX-License-Identifier: ISC
// Copyright (c) 2016, Autonomous Vehicle System Lab, University of Colorado at Boulder
// Copyright (c) 2025, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

#ifndef SC_ENERGY_MOMENTUM_MESSAGE_H
#define SC_ENERGY_MOMENTUM_MESSAGE_H

/*! @brief This structure is used in the messaging system to communicate what the
 state of the vehicle is currently.*/
typedef struct {
    double spacecraftOrbEnergy;           //!< [J] Total orbital kinetic energy
    double spacecraftRotEnergy;           //!< [J] Total rotational energy
    double spacecraftOrbAngMomPntN_N[3];  //!< [kg m^2/s] Total orbital angular momentum about N in N frame components
    double
        spacecraftRotAngMomPntC_N[3];  //!< [kg m^2/s] Total rotational angular momentum about C in N frame components
} SCEnergyMomentumMsgPayload;

#endif
