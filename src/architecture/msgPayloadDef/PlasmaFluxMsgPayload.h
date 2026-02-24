// SPDX-License-Identifier: ISC
// Copyright (c) 2021, Autonomous Vehicle System Lab, University of Colorado at Boulder
// Copyright (c) 2025, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

#ifndef PLASMA_FLUX_MESSAGE_H_
#define PLASMA_FLUX_MESSAGE_H_

#define MAX_PLASMA_FLUX_SIZE 50

//!@brief space weather ambient plasma flux message definition.
typedef struct {
    double meanElectronFlux[MAX_PLASMA_FLUX_SIZE];  //!< [m^-2 s^-1 sr^-2 eV^-1] differential flux
    double meanIonFlux[MAX_PLASMA_FLUX_SIZE];       //!< [m^-2 s^-1 sr^-2 eV^-1] differential flux
    double energies[MAX_PLASMA_FLUX_SIZE];          //!< [eV]
} PlasmaFluxMsgPayload;

#endif
