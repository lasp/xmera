// SPDX-License-Identifier: ISC
// Copyright (c) 2025, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

#ifndef IMG_DIAGNOSTIC_MSG_H
#define IMG_DIAGNOSTIC_MSG_H

/*! @brief Message to store image diagnostics returned and caught by Cielim Interface.*/
typedef struct {
    double centerOfBrightness[2];         //!< [pix] coordinates in the camera frame of the center of brightness
    double areaOfInterestCenter[2];       //!< [pix] coordinates of the center of the area of interest
    double areaOfInterestWidthHeight[2];  //!< [pix] Width and Height of the area of interest
    double totalBrightPixels;             //!< [-] Total number of bright pixels detected
    double threshold;  //!< [0-255] threshold for a bright pixel to count (values below threshold are zeroed)
    double coverage;   //!< [-] ratio of bright pixels in the area of interest to the total bright pixels
} ImageDiagnosticsPayload;

#endif /* IMG_DIAGNOSTIC_MSG_H */
