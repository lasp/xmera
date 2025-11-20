/*
 ISC License

 Copyright (c) 2025 University of Colorado at Boulder

 Permission to use, copy, modify, and/or distribute this software for any
 purpose with or without fee is hereby granted, provided that the above
 copyright notice and this permission notice appear in all copies.

 THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
 WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
 MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
 ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
 WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
 ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
 OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.

 */

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
