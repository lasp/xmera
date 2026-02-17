// SPDX-License-Identifier: ISC
// Copyright (c) 2025, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

#ifndef IMAGE_MSG_H
#define IMAGE_MSG_H

#define MAX_FILENAME_LENGTH 10000

/*! @brief Structure used to define the image */

#include "architecture/utilities/macroDefinitions.h"

/*! @brief Structure used to define the output definition for attitude guidance*/
typedef struct {
    uint64_t timeTag;           //!< --[ns]   Current vehicle time-tag associated with measurements*/
    int valid;                  //!< --  A valid image is present for 1, 0 if not*/
    int64_t cameraID;           //!< -- [-]   ID of the camera that took the snapshot*/
    void* imagePointer;         //!< -- Pointer to the image
    int32_t imageBufferLength;  //!< -- Length of the buffer for recasting
    int8_t imageType;           //!< -- Number of channels in each pixel, RGB = 3, RGBA = 4
} CameraImageMsgPayload;

#endif
