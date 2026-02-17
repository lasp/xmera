// SPDX-License-Identifier: ISC
// Copyright (c) 2021, Autonomous Vehicle System Lab, University of Colorado at Boulder
// Copyright (c) 2025, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

#ifndef RECONFIG_BURN_ARRAY_INFO_H
#define RECONFIG_BURN_ARRAY_INFO_H

#define MAX_BURN_CNT 3

#include "ReconfigBurnInfoMsgPayload.h"

//! @brief Container for the orbit reconfiguration burn information.
typedef struct {
    ReconfigBurnInfoMsgPayload burnArray[MAX_BURN_CNT];  //!< array of burn info messages
} ReconfigBurnArrayInfoMsgPayload;

#endif
