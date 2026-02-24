// SPDX-License-Identifier: ISC
// Copyright (c) 2016, Autonomous Vehicle System Lab, University of Colorado at Boulder
// Copyright (c) 2025, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

#ifndef solarFluxSimMsg_h
#define solarFluxSimMsg_h

//!@brief Solar flux message definition.
typedef struct {
    double flux;  //!< [W/m2] at s/c position
} SolarFluxMsgPayload;

#endif /* solarFluxSimMsg_h */
