// SPDX-License-Identifier: ISC
// Copyright (c) 2016, Autonomous Vehicle System Lab, University of Colorado at Boulder
// Copyright (c) 2025, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

#ifndef fuelTankSimMsg_h
#define fuelTankSimMsg_h

/*! @brief Structure used to define the individual fuel tank mass property message*/
typedef struct {
    double fuelMass;     //!< [kg], current fuel mass
    double fuelMassDot;  //!< [kg/s], current rate of change of fuel mass
    double maxFuelMass;  //!< [kg], maximum fuel mass capacity
} FuelTankMsgPayload;

#endif /* fuelTankSimMsg_h */
