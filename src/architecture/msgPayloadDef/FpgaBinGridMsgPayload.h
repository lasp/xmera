#ifndef FPGA_BIN_GRID_MSG_PAYLOAD_H
#define FPGA_BIN_GRID_MSG_PAYLOAD_H

#include <stdint.h>

// SWIG cannot parse digit separators, so clang-format must not add them here.
// clang-format off
#define FPGA_BIN_GRID_MAX_REGIONS 3008  //!< 64 x 47 regions: a 4096 x 3000 image with 64 px regions
// clang-format on

/*! @brief One ROI region of the bin grid. */
typedef struct {
    uint32_t x;              //!< [px] Column of the region top-left pixel
    uint32_t y;              //!< [px] Row of the region top-left pixel
    uint32_t litPixelCount;  //!< [-]  Number of above-threshold pixels in the region
} FpgaBinRegion;

/*! @brief All ROI regions of the image, with position and above-threshold pixel count.
 *  The first numRegions entries of regions are valid, in row-major order.
 *  Regions at the right and bottom edges can be smaller than regionSize.
 */
typedef struct {
    uint64_t timeTag;                                  //!< [ns] Time tag from upstream camera emulator
    uint32_t regionSize;                               //!< [px] Region size: 64, 128, or 256 pixels
    uint32_t numRegionRows;                            //!< [-]  = ceil(image height / regionSize)
    uint32_t numRegionCols;                            //!< [-]  = ceil(image width / regionSize)
    uint32_t numRegions;                               //!< [-]  = numRegionRows * numRegionCols
    FpgaBinRegion regions[FPGA_BIN_GRID_MAX_REGIONS];  //!< [-]  Regions in row-major order
} FpgaBinGridMsgPayload;

#endif
