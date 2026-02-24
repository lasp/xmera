#ifndef FPGA_ROW_COL_SUM_MSG_PAYLOAD_H
#define FPGA_ROW_COL_SUM_MSG_PAYLOAD_H

#include <stdint.h>

/*! @brief Row/col accumulators: count of pixels above threshold per row and column.
 *  Values fit in uint16_t (max = max(width, height) <= 65535).
 */
typedef struct {
    uint64_t timeTag;            //!< [ns]   Time tag from upstream camera emulator
    uint32_t numRows;            //!< [-]    = image height; length of rowSumPointer array
    uint32_t numCols;            //!< [-]    = image width;  length of colSumPointer array
    void* rowSumPointer;         //!< Pointer to uint16_t[numRows]
    void* colSumPointer;         //!< Pointer to uint16_t[numCols]
    int32_t rowSumBufferLength;  //!< [bytes] = numRows * sizeof(uint16_t)
    int32_t colSumBufferLength;  //!< [bytes] = numCols * sizeof(uint16_t)
} FpgaRowColSumMsgPayload;

#endif
