#ifndef FPGA_BINS_MSG_PAYLOAD_H
#define FPGA_BINS_MSG_PAYLOAD_H

#include <stdint.h>

#define FPGA_ROI_TOP_COUNT 8

/*! @brief A single ranked region-of-interest entry. */
typedef struct {
    uint16_t col;    //!< [-] Column index of region top-left corner (in region units)
    uint16_t row;    //!< [-] Row index of region top-left corner (in region units)
    uint32_t count;  //!< [-] Count of pixels above threshold in this region
} FpgaRoiEntry;

/*! @brief Top-ranked ROI regions sorted descending by above-threshold pixel count. */
typedef struct {
    uint64_t timeTag;                          //!< [ns]   Time tag from upstream camera emulator
    uint32_t regionSize;                       //!< [px]   Region size: 64, 128, or 256 pixels
    uint32_t numValidRegions;                  //!< [-]    Number of valid entries (<= FPGA_ROI_TOP_COUNT)
    FpgaRoiEntry topBins[FPGA_ROI_TOP_COUNT];  //!< [-]    Top regions sorted descending by count
} FpgaBinsMsgPayload;

#endif
