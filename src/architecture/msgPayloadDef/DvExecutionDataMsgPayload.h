// SPDX-License-Identifier: ISC
// Copyright (c) 2016, Autonomous Vehicle System Lab, University of Colorado at Boulder
// Copyright (c) 2025, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

#ifndef DV_EXECUTION_DATA_H
#define DV_EXECUTION_DATA_H

#include "stdint.h"

/*! @brief DV executation data structure */
typedef struct {
    uint32_t burnExecuting; /*!< [-] Flag indicating whether burn is executing*/
    uint32_t burnComplete;  /*!< [-] Flag indicating whether the burn is complete */
} DvExecutionDataMsgPayload;

#endif
