#ifndef FPGA_IMAGE_PIPELINE_H
#define FPGA_IMAGE_PIPELINE_H

#include <architecture/_GeneralModuleFiles/sys_model.h>
#include <architecture/messaging/messaging.h>
#include <architecture/msgPayloadDef/CameraImageMsgPayload.h>
#include <architecture/msgPayloadDef/FpgaBinGridMsgPayload.h>
#include <architecture/msgPayloadDef/FpgaBinsMsgPayload.h>
#include <architecture/msgPayloadDef/FpgaPipelineConfigMsgPayload.h>
#include <architecture/msgPayloadDef/FpgaRawImageMsgPayload.h>
#include <architecture/msgPayloadDef/FpgaRowColSumMsgPayload.h>
#include <architecture/msgPayloadDef/FpgaThreshImageMsgPayload.h>

#include <cstdint>
#include <string>
#include <vector>

/*! @brief FPGA image processing pipeline simulation module.
 *
 *  Simulates the FPGA image processing pipeline of a star-tracker/MAC camera instrument.
 *  Every intermediate data product is published as an output message for verification.
 *
 *  Pipeline stages:
 *    1. Pixel calibration pre-processing
 *    2. Separable box blur
 *    3. Binary threshold (1-bit packing)
 *    4. Row/column above-threshold counts and intensity sums
 *    5. Region-of-interest (ROI) bin grid and ranking (top 8)
 */
class FpgaImagePipeline : public SysModel {
public:
    void reset(uint64_t callTime) override;
    void updateState(uint64_t callTime) override;

    // --- Configuration setters/getters ---
    void setImageWidth(uint32_t w);
    uint32_t getImageWidth() const;

    void setImageHeight(uint32_t h);
    uint32_t getImageHeight() const;

    void setKernelSize(uint8_t k);
    uint8_t getKernelSize() const;  //!< Must be 5, 7, or 9

    void setThreshold(uint16_t t);
    uint16_t getThreshold() const;

    void setRoiRegionSize(uint32_t s);
    uint32_t getRoiRegionSize() const;  //!< Must be 64, 128, or 256

    void setCalibRegA(uint16_t v);
    uint16_t getCalibRegA() const;

    void setCalibRegB(uint16_t v);
    uint16_t getCalibRegB() const;

    void setCalibRegC(uint16_t v);
    uint16_t getCalibRegC() const;

    void setCalibRegD(uint16_t v);
    uint16_t getCalibRegD() const;

    void setCalibEnabled(bool e);
    bool getCalibEnabled() const;

    void setCalibImageFile(std::string const &path);
    void setImageFileName(std::string const &path);  //!< Load image from disk each updateState()

    void setSaveImages(bool save);
    bool getSaveImages() const;

    void setSaveDir(std::string const &dir);
    std::string getSaveDir() const;

    // --- Internal buffer accessors (for testing/verification) ---
    uint16_t getRawPixel(uint32_t idx) const;
    uint16_t getBlurPixel(uint32_t idx) const;
    bool getThreshBit(uint32_t idx) const;
    uint16_t getRowSum(uint32_t row) const;
    uint16_t getColSum(uint32_t col) const;
    uint32_t getRowIntensity(uint32_t row) const;  //!< Sum of blurred values above threshold in a row
    uint32_t getColIntensity(uint32_t col) const;  //!< Sum of blurred values above threshold in a column
    uint32_t getNumRegionRows() const;             //!< Number of ROI region rows, partial bottom region included
    uint32_t getNumRegionCols() const;             //!< Number of ROI region columns, partial right region included
    uint32_t getBinCount(uint32_t regionRow, uint32_t regionCol) const;  //!< Above-threshold count in a region

    // --- Message interfaces ---
    ReadFunctor<CameraImageMsgPayload> imageInMsg;  //!< Optional: linked to camera emulator

    Message<FpgaRawImageMsgPayload> rawImageOutMsg;        //!< After calibration pre-processing
    Message<FpgaRawImageMsgPayload> blurredImageOutMsg;    //!< After box blur
    Message<FpgaThreshImageMsgPayload> threshImageOutMsg;  //!< Binary threshold result
    Message<FpgaRowColSumMsgPayload> rowColSumOutMsg;      //!< Row/col accumulators
    Message<FpgaBinsMsgPayload> roiOutMsg;                 //!< Top-8 ROI regions
    Message<FpgaBinGridMsgPayload> binGridOutMsg;          //!< Position and count of every ROI region
    Message<FpgaPipelineConfigMsgPayload> configOutMsg;    //!< Ancillary config snapshot


private:
    // --- Configuration state ---
    uint32_t imageWidth{4'096};
    uint32_t imageHeight{3'000};
    uint8_t kernelSize{5};
    uint16_t threshold{0};
    uint32_t roiRegionSize{64};
    uint16_t calibRegA{0};
    uint16_t calibRegB{0};
    uint16_t calibRegC{0};
    uint16_t calibRegD{0};
    bool calibEnabled{false};
    bool saveImages{false};
    std::string calibImageFile{};
    std::string imageFileName{};
    std::string saveDir{};

    // --- Internal buffers (allocated in reset()) ---
    std::vector<uint16_t> rawBuf;           //!< Calibration-preprocessed pixels (12-bit in uint16)
    std::vector<uint16_t> blurBuf;          //!< Final blurred pixels (16-bit, divided)
    std::vector<uint8_t> threshBuf;         //!< 1-bit packed threshold image
    std::vector<uint16_t> rowSumBuf;        //!< Above-threshold count per row
    std::vector<uint16_t> colSumBuf;        //!< Above-threshold count per column
    std::vector<uint32_t> rowIntensityBuf;  //!< Sum of blurred values above threshold per row
    std::vector<uint32_t> colIntensityBuf;  //!< Sum of blurred values above threshold per column
    std::vector<FpgaBinRegion> regionBuf;   //!< Position and above-threshold count per ROI region, row-major
    std::vector<uint16_t> calibImage;       //!< Loaded calibration image (empty if not loaded)
    uint32_t numRegionRows{0};              //!< = ceil(imageHeight / roiRegionSize)
    uint32_t numRegionCols{0};              //!< = ceil(imageWidth / roiRegionSize)

    void allocateBuffers();  //!< Size all internal buffers to the current image dimensions

    // --- Pipeline stage methods ---
    bool loadImage(uint64_t callTime, uint64_t &timeTagOut);
    void applyCalibration();
    void applyBlurAndThreshold();
    void computeRowColSums();
    void computeRoi(FpgaBinsMsgPayload &roi);
    void publishOutputs(uint64_t callTime, uint64_t imageTimeTag, FpgaBinsMsgPayload &roi);
    void saveDataToDisk(uint64_t timeTagNs, FpgaBinsMsgPayload const &roi);

    // --- Helpers ---
    static uint16_t clamp12(int32_t v);
    static uint8_t blurShift(uint8_t kernelSize);
};

#endif
