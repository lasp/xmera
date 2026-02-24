#ifndef FPGA_THRESH_IMAGE_MSG_PAYLOAD_H
#define FPGA_THRESH_IMAGE_MSG_PAYLOAD_H

#include <stdint.h>

/*! @brief 1-bit per pixel packed MSB-first into bytes: ceil(width*height/8) bytes total. */
typedef struct {
    uint64_t timeTag;           //!< [ns]   Time tag from upstream camera emulator
    uint32_t width;             //!< [px]   Image width
    uint32_t height;            //!< [px]   Image height
    uint16_t threshold;         //!< [-]    Threshold value applied
    void* imagePointer;         //!< Pointer to bit-packed uint8_t buffer
    int32_t imageBufferLength;  //!< [bytes] Bytes in imagePointer (= ceil(width*height/8))
} FpgaThreshImageMsgPayload;

#endif
