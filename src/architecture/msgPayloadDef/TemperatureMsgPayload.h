// SPDX-License-Identifier: ISC
// Copyright (c) 2021, Autonomous Vehicle System Lab, University of Colorado at Boulder
// Copyright (c) 2025, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

#ifndef TEMPERATURE_H
#define TEMPERATURE_H

/*! @brief Message for reporting the power consumed produced or consumed by a module.*/
typedef struct {
    double temperature;  //!< [Celsius] current temperature
} TemperatureMsgPayload;

#endif  // TEMPERATURE_H
