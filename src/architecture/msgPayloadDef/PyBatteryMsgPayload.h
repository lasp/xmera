// SPDX-License-Identifier: ISC
// Copyright (c) 2016, Autonomous Vehicle System Lab, University of Colorado at Boulder
// Copyright (c) 2025, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

#ifndef PY_BATTERY_OUT_MSG_H
#define PY_BATTERY_OUT_MSG_H

#include <stdint.h>
#include <string>

//! @brief Container for all battery output data
/*! This structure contains all data output by the python battery module*/
typedef struct {
    double stateOfCharge;          //!< [%] Battery state of charge as %-full
    double stateOfChargeAh;        //!< [Ah] Battery state of charge un-scaled
    double solarArrayTemperature;  //!< [K] Temperature of solar arrays
    double busVoltage;             //!< [V] Bus voltage
    double batteryCurrent;         //!< [A] Total current flowing through battery
    double solarArrayCurrent;      //!< [A] Current sent to battery from solar arrays
    double batteryEMF;             //!< [V] Electro-motive force of battery at this current form look-up table
    double batteryESR;             //!< [Ohms] Equivalent Series Resistance of battery from look-up table
    double batteryVoltage;         //!< [V] total voltage across battery
} PyBatteryMsgPayload;

#endif
