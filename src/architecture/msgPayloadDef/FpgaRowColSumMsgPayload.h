#ifndef FPGA_ROW_COL_SUM_MSG_PAYLOAD_H
#define FPGA_ROW_COL_SUM_MSG_PAYLOAD_H

#include <stdint.h>

// SWIG cannot parse digit separators, so clang-format must not add them here.
// clang-format off
#define FPGA_ROW_COL_MAX_ROWS 3000  //!< Maximum image height in pixels
#define FPGA_ROW_COL_MAX_COLS 4096  //!< Maximum image width in pixels
// clang-format on

/*! @brief Row/col accumulators per row and column, for pixels above threshold.
 *  Only the first numRows row entries and the first numCols column entries are valid.
 */
typedef struct {
    uint64_t timeTag;                         //!< [ns] Time tag from upstream camera emulator
    uint32_t numRows;                         //!< [-]  = image height, <= FPGA_ROW_COL_MAX_ROWS
    uint32_t numCols;                         //!< [-]  = image width, <= FPGA_ROW_COL_MAX_COLS
    uint16_t rowSums[FPGA_ROW_COL_MAX_ROWS];  //!< [-]  Above-threshold pixel count per row
    uint16_t colSums[FPGA_ROW_COL_MAX_COLS];  //!< [-]  Above-threshold pixel count per column
} FpgaRowColSumMsgPayload;

#endif
