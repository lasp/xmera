#ifndef FPGA_PIPELINE_CONFIG_MSG_PAYLOAD_H
#define FPGA_PIPELINE_CONFIG_MSG_PAYLOAD_H

#include <stdbool.h>
#include <stdint.h>

#define FPGA_CALIB_REF_LEN 512

/*! @brief Ancillary metadata snapshot of the pipeline configuration. */
typedef struct {
    uint64_t timeTag;                        //!< [ns]   Time tag from upstream camera emulator
    uint32_t imageWidth;                     //!< [px]   Configured image width
    uint32_t imageHeight;                    //!< [px]   Configured image height
    uint8_t kernelSize;                      //!< [-]    Box blur kernel size: 5, 7, or 9
    uint16_t thresholdValue;                 //!< [-]    Binary threshold value
    uint32_t roiRegionSize;                  //!< [px]   ROI region size: 64, 128, or 256
    uint16_t calibRegA;                      //!< [-]    Calibration register A
    uint16_t calibRegB;                      //!< [-]    Calibration register B
    uint16_t calibRegC;                      //!< [-]    Calibration register C
    uint16_t calibRegD;                      //!< [-]    Calibration register D
    bool calibEnabled;                       //!< [-]    True if calibration pre-processing is enabled
    char calibImageRef[FPGA_CALIB_REF_LEN];  //!< [-]    File path or identifier of calibration image
} FpgaPipelineConfigMsgPayload;

#endif
