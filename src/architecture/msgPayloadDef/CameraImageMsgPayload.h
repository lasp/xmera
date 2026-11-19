// SPDX-License-Identifier: ISC
// Copyright (c) 2025, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

#ifndef IMAGE_MSG_H
#define IMAGE_MSG_H

#include "architecture/utilities/macroDefinitions.h"

#include <cstdint>

/*! How the image buffer is encoded. */
enum class ImageEncoding : uint8_t {
    PNG,  //!< A PNG file; it describes its own size, depth and channels
    RAW,  //!< Raw pixel values described by width, height, colorLayout and bitsPerPixel
};

/*! Which color each value in the buffer represents. */
enum class ColorLayout : uint8_t {
    GRAYSCALE,   //!< One channel per pixel
    BAYER_RGGB,  //!< One channel per pixel, color filter mosaic starting with red, green / green, blue (not used yet)
    BAYER_BGGR,  //!< One channel per pixel, color filter mosaic starting with blue, green / green, red (not used yet)
    BAYER_GRBG,  //!< One channel per pixel, color filter mosaic starting with green, red / blue, green (not used yet)
    BAYER_GBRG,  //!< One channel per pixel, color filter mosaic starting with green, blue / red, green (not used yet)
    RGB,         //!< Three channels per pixel: red, green, blue
};

/*! @brief An image from a camera. */
struct CameraImageMsgPayload {
    uint64_t timeTag;           //!< Time the image was taken in nanoseconds
    uint8_t valid;              //!< Is 1 if an image is present, 0 if not
    int64_t cameraID;           //!< ID of the camera that took the image
    void* imagePointer;         //!< Pointer to the image buffer
    int32_t imageBufferLength;  //!< Length of the image buffer in bytes
    ImageEncoding encoding;     //!< How the image buffer is encoded
    ColorLayout colorLayout;    //!< Which color each value represents
    uint32_t width;             //!< Image width in pixels
    uint32_t height;            //!< Image height in pixels
    uint8_t bitsPerPixel;       //!< Largest number of bits a value can use
};

#endif
