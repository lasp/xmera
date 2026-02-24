// SPDX-License-Identifier: ISC
// Copyright (c) 2019, Autonomous Vehicle System Lab, University of Colorado at Boulder
// Copyright (c) 2025, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

#ifndef _TAM_SENSOR_BODY_MESSAGE_H
#define _TAM_SENSOR_BODY_MESSAGE_H

/*! @brief Output structure for TAM measurements*/
typedef struct {
    double tam_B[3];  //!< [Tesla] TAM measurements in vehicle body frame
} TAMSensorBodyMsgPayload;

#endif
