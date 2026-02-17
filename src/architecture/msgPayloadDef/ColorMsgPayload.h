// SPDX-License-Identifier: ISC
// Copyright (c) 2023, Autonomous Vehicle System Lab, University of Colorado at Boulder
// Copyright (c) 2025, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

#ifndef _COLOR_MESSAGE_H
#define _COLOR_MESSAGE_H

/*! @brief Structure used to define RGBA color */
typedef struct {
    int colorRGBA[4];  //!< [-] 0-255 Color values in RGBA format
} ColorMsgPayload;

#endif
