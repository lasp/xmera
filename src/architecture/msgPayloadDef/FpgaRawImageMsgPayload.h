#ifndef FPGA_RAW_IMAGE_MSG_PAYLOAD_H
#define FPGA_RAW_IMAGE_MSG_PAYLOAD_H

#include <stdint.h>

/*! @brief Used for both the calibration-preprocessed ("raw") image and the blurred image.
 *  imagePointer points to a uint16_t array with width*height elements, 12-bit values in lower bits.
 */
typedef struct {
    uint64_t timeTag;           //!< [ns]   Time tag from upstream camera emulator
    uint32_t width;             //!< [px]   Image width
    uint32_t height;            //!< [px]   Image height
    void* imagePointer;         //!< Pointer to uint16_t buffer (width*height elements)
    int32_t imageBufferLength;  //!< [bytes] Bytes in imagePointer (= width*height*sizeof(uint16_t))
} FpgaRawImageMsgPayload;

#endif
