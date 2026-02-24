// SPDX-License-Identifier: ISC
// Copyright (c) 2019, Autonomous Vehicle System Lab, University of Colorado at Boulder
// Copyright (c) 2025, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

#ifndef _TAM_SENSOR_MESSAGE_H
#define _TAM_SENSOR_MESSAGE_H

//! @brief Simulated TAM Sensor output message definition.
typedef struct {
    double tam_S[3];  //!< [T] Magnetic field measurements in sensor frame
} TAMSensorMsgPayload;

#endif
