// SPDX-License-Identifier: ISC
// Copyright (c) 2016, Autonomous Vehicle System Lab, University of Colorado at Boulder
// Copyright (c) 2025, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

#ifndef _ACC_PKT_DATA_MESSAGE_H
#define _ACC_PKT_DATA_MESSAGE_H

#define MAX_ACC_BUF_PKT 120

#include "AccPktDataMsgPayload.h"

/*! @brief Structure used to define accelerometer package data */
typedef struct {
    AccPktDataMsgPayload accPkts[MAX_ACC_BUF_PKT];  //!< [-] Accelerometer buffer read in
} AccDataMsgPayload;

#endif
