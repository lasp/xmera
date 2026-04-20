#include "fpgaImagePipeline.h"

#include <algorithm>
#include <cstring>
#include <fstream>
#include <numeric>
#include <stdexcept>

#include <opencv2/imgcodecs.hpp>

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------

uint16_t FpgaImagePipeline::clamp12(int32_t v) {
    if (v < 0) return 0;
    if (v > 4095) return 4095;
    return static_cast<uint16_t>(v);
}

uint8_t FpgaImagePipeline::blurShift(const uint8_t kernelSize) {
    switch (kernelSize) {
        case 5:
            return 1;
        case 7:
            return 2;
        case 9:
            return 3;
        default:
            return 1;
    }
}

// ---------------------------------------------------------------------------
// Configuration setters / getters
// ---------------------------------------------------------------------------

void FpgaImagePipeline::setImageWidth(const uint32_t w) { this->imageWidth = w; }
uint32_t FpgaImagePipeline::getImageWidth() const { return this->imageWidth; }

void FpgaImagePipeline::setImageHeight(const uint32_t h) { this->imageHeight = h; }
uint32_t FpgaImagePipeline::getImageHeight() const { return this->imageHeight; }

void FpgaImagePipeline::setKernelSize(const uint8_t k) { this->kernelSize = k; }
uint8_t FpgaImagePipeline::getKernelSize() const { return this->kernelSize; }

void FpgaImagePipeline::setThreshold(const uint16_t t) { this->threshold = t; }
uint16_t FpgaImagePipeline::getThreshold() const { return this->threshold; }

void FpgaImagePipeline::setRoiRegionSize(const uint32_t s) { this->roiRegionSize = s; }
uint32_t FpgaImagePipeline::getRoiRegionSize() const { return this->roiRegionSize; }

void FpgaImagePipeline::setCalibRegA(const uint16_t v) { this->calibRegA = v; }
uint16_t FpgaImagePipeline::getCalibRegA() const { return this->calibRegA; }

void FpgaImagePipeline::setCalibRegB(const uint16_t v) { this->calibRegB = v; }
uint16_t FpgaImagePipeline::getCalibRegB() const { return this->calibRegB; }

void FpgaImagePipeline::setCalibRegC(const uint16_t v) { this->calibRegC = v; }
uint16_t FpgaImagePipeline::getCalibRegC() const { return this->calibRegC; }

void FpgaImagePipeline::setCalibRegD(const uint16_t v) { this->calibRegD = v; }
uint16_t FpgaImagePipeline::getCalibRegD() const { return this->calibRegD; }

void FpgaImagePipeline::setCalibEnabled(const bool e) { this->calibEnabled = e; }
bool FpgaImagePipeline::getCalibEnabled() const { return this->calibEnabled; }

void FpgaImagePipeline::setCalibImageFile(const std::string& path) { this->calibImageFile = path; }
void FpgaImagePipeline::setImageFileName(const std::string& path) { this->imageFileName = path; }

void FpgaImagePipeline::setSaveImages(const bool save) { this->saveImages = save; }
bool FpgaImagePipeline::getSaveImages() const { return this->saveImages; }
void FpgaImagePipeline::setSaveDir(const std::string& dir) { this->saveDir = dir; }
std::string FpgaImagePipeline::getSaveDir() const { return this->saveDir; }

// ---------------------------------------------------------------------------
// Internal buffer accessors (for testing/verification)
// ---------------------------------------------------------------------------

uint16_t FpgaImagePipeline::getRawPixel(const uint32_t idx) const {
    if (idx >= this->rawBuf.size()) {
        throw std::out_of_range("getRawPixel: index out of range");
    }
    return this->rawBuf[idx];
}

uint16_t FpgaImagePipeline::getBlurPixel(const uint32_t idx) const {
    if (idx >= this->blurBuf.size()) {
        throw std::out_of_range("getBlurPixel: index out of range");
    }
    return this->blurBuf[idx];
}

bool FpgaImagePipeline::getThreshBit(const uint32_t idx) const {
    const uint32_t byteIdx = idx / 8;
    if (byteIdx >= this->threshBuf.size()) {
        throw std::out_of_range("getThreshBit: index out of range");
    }
    const uint8_t bitIdx = static_cast<uint8_t>(7u - (idx % 8u));
    return (this->threshBuf[byteIdx] >> bitIdx) & 0x01;
}

uint16_t FpgaImagePipeline::getRowSum(const uint32_t row) const {
    if (row >= this->rowSumBuf.size()) {
        throw std::out_of_range("getRowSum: index out of range");
    }
    return this->rowSumBuf[row];
}

uint16_t FpgaImagePipeline::getColSum(const uint32_t col) const {
    if (col >= this->colSumBuf.size()) {
        throw std::out_of_range("getColSum: index out of range");
    }
    return this->colSumBuf[col];
}

// ---------------------------------------------------------------------------
// reset()
// ---------------------------------------------------------------------------

void FpgaImagePipeline::reset(uint64_t /*callTime*/) {
    // Validate kernel size
    if (this->kernelSize != 5 && this->kernelSize != 7 && this->kernelSize != 9) {
        throw std::invalid_argument("FpgaImagePipeline: kernelSize must be 5, 7, or 9");
    }

    // Validate ROI region size
    if (this->roiRegionSize != 64 && this->roiRegionSize != 128 && this->roiRegionSize != 256) {
        throw std::invalid_argument("FpgaImagePipeline: roiRegionSize must be 64, 128, or 256");
    }

    const size_t n = static_cast<size_t>(this->imageWidth) * this->imageHeight;
    const size_t threshBytes = (n + 7) / 8;

    this->rawBuf.assign(n, 0);
    this->blurBuf.assign(n, 0);
    this->threshBuf.assign(threshBytes, 0);
    this->rowSumBuf.assign(this->imageHeight, 0);
    this->colSumBuf.assign(this->imageWidth, 0);

    // Load calibration image if configured
    this->calibImage.clear();
    if (!this->calibImageFile.empty()) {
        cv::Mat calibMat = cv::imread(this->calibImageFile, cv::IMREAD_ANYDEPTH | cv::IMREAD_GRAYSCALE);
        if (calibMat.empty()) {
            bskLogger.bskLog(BSK_WARNING,
                             "FpgaImagePipeline: failed to load calibration image from '%s'",
                             this->calibImageFile.c_str());
        } else {
            const uint32_t cw = static_cast<uint32_t>(calibMat.cols);
            const uint32_t ch = static_cast<uint32_t>(calibMat.rows);
            if (cw != this->imageWidth || ch != this->imageHeight) {
                throw std::invalid_argument(
                    "FpgaImagePipeline: calibration image dimensions do not match this->imageWidth/this->imageHeight");
            }
            if (calibMat.depth() != CV_16U) {
                calibMat.convertTo(calibMat, CV_16U);
            }
            this->calibImage.resize(n);
            for (uint32_t r = 0; r < ch; r++) {
                for (uint32_t c = 0; c < cw; c++) {
                    this->calibImage[r * cw + c] = calibMat.at<uint16_t>(r, c);
                }
            }
        }
    }
}

// ---------------------------------------------------------------------------
// updateState()
// ---------------------------------------------------------------------------

void FpgaImagePipeline::updateState(uint64_t callTime) {
    uint64_t imageTimeTag = 0;
    if (!loadImage(callTime, imageTimeTag)) {
        return;
    }

    this->applyCalibration();
    this->applyBlurAndThreshold();
    this->computeRowColSums();

    FpgaBinsMsgPayload roi{};
    this->computeRoi(roi);

    this->publishOutputs(callTime, imageTimeTag, roi);

    if (this->saveImages && !this->saveDir.empty()) {
        this->saveDataToDisk(imageTimeTag, roi);
    }
}

// ---------------------------------------------------------------------------
// loadImage()
// ---------------------------------------------------------------------------

bool FpgaImagePipeline::loadImage(const uint64_t callTime, uint64_t& timeTagOut) {
    bool imageSourceIsConfigured = false;

    if (!this->imageFileName.empty()) {
        cv::Mat img = cv::imread(this->imageFileName, cv::IMREAD_ANYDEPTH | cv::IMREAD_GRAYSCALE);
        if (img.empty()) {
            bskLogger.bskLog(
                BSK_WARNING, "FpgaImagePipeline: failed to load image from '%s'", this->imageFileName.c_str());
        }

        // Normalise to 16-bit so img.at<uint16_t> is always valid.
        // IMREAD_ANYDEPTH may return CV_8UC1, CV_32FC1, etc.
        if (img.depth() == CV_8U) {
            // Scale 8-bit [0,255] → 12-bit equivalent [0,4080] so the pipeline's
            // threshold and blur parameters have the same meaning as for a real
            // 12-bit sensor (factor 16 = 4096/256).
            img.convertTo(img, CV_16U, 16.0);
        } else if (img.depth() != CV_16U) {
            img.convertTo(img, CV_16U);
        }

        const auto fileWidth = static_cast<uint32_t>(img.cols);
        // If dimensions differ from allocated buffers, resize everything
        if (const auto fileHeight = static_cast<uint32_t>(img.rows);
            fileWidth != this->imageWidth || fileHeight != this->imageHeight) {
            bskLogger.bskLog(BSK_WARNING,
                             "FpgaImagePipeline: image file dimensions %ux%u differ from "
                             "configured %ux%u; re-allocating buffers",
                             fileWidth,
                             fileHeight,
                             this->imageWidth,
                             this->imageHeight);
            this->imageWidth = fileWidth;
            this->imageHeight = fileHeight;
            const size_t newN = static_cast<size_t>(this->imageWidth) * this->imageHeight;
            this->rawBuf.assign(newN, 0);
            this->blurBuf.assign(newN, 0);
            this->threshBuf.assign((newN + 7) / 8, 0);
            this->rowSumBuf.assign(this->imageHeight, 0);
            this->colSumBuf.assign(this->imageWidth, 0);
        }

        for (uint32_t r = 0; r < this->imageHeight; r++) {
            for (uint32_t c = 0; c < this->imageWidth; c++) {
                this->rawBuf[r * this->imageWidth + c] = std::min(img.at<uint16_t>(r, c), uint16_t{4095});
            }
        }
        timeTagOut = callTime;
        imageSourceIsConfigured = true;

    } else if (imageInMsg.isLinked()) {
        const CameraImageMsgPayload imgMsg = imageInMsg();
        if (!imgMsg.valid || imgMsg.imagePointer == nullptr) {
            bskLogger.bskLog(BSK_WARNING, "FpgaImagePipeline: imageInMsg has invalid or null image");
        }

        const size_t n = static_cast<size_t>(this->imageWidth) * this->imageHeight;
        const auto src = reinterpret_cast<const uint16_t*>(imgMsg.imagePointer);
        for (size_t i = 0; i < n; i++) {
            this->rawBuf[i] = src[i] & 0x0FFF;
        }
        timeTagOut = imgMsg.timeTag;
    } else {
        bskLogger.bskLog(BSK_WARNING, "FpgaImagePipeline: no image source (set imageFileName or link imageInMsg)");
    }
    return imageSourceIsConfigured;
}

// ---------------------------------------------------------------------------
// applyCalibration()
// ---------------------------------------------------------------------------

void FpgaImagePipeline::applyCalibration() {
    if (!this->calibEnabled || this->calibImage.empty()) {
        return;  // this->rawBuf already holds the uncalibrated 12-bit pixels
    }

    const size_t n = static_cast<size_t>(this->imageWidth) * this->imageHeight;
    for (size_t i = 0; i < n; i++) {
        uint16_t raw = this->rawBuf[i];
        const uint16_t calibWord = this->calibImage[i];
        const uint8_t opCode = static_cast<uint8_t>((calibWord >> 12) & 0x0F);
        const uint16_t calibVal = calibWord & 0x0FFF;

        // Select register value based on opCode
        uint16_t regVal = 0;
        switch (opCode) {
            case 0x1:
            case 0x6:
            case 0xb:
                regVal = this->calibRegA;
                break;
            case 0x2:
            case 0x7:
            case 0xc:
                regVal = this->calibRegB;
                break;
            case 0x3:
            case 0x8:
            case 0xd:
                regVal = this->calibRegC;
                break;
            case 0x4:
            case 0x9:
            case 0xe:
                regVal = this->calibRegD;
                break;
            default:
                regVal = calibVal;
                break;
        }

        switch (opCode) {
            case 0x0:
                break;  // pass
            case 0x1:
            case 0x2:
            case 0x3:
            case 0x4:
                raw = regVal;
                break;  // set to reg
            case 0x5:
                raw = calibVal;
                break;  // set to literal
            case 0x6:
            case 0x7:
            case 0x8:
            case 0x9:
                raw = clamp12(static_cast<int32_t>(raw) + static_cast<int32_t>(regVal));
                break;
            case 0xa:
                raw = clamp12(static_cast<int32_t>(raw) + static_cast<int32_t>(calibVal));
                break;
            case 0xb:
            case 0xc:
            case 0xd:
            case 0xe:
                raw = clamp12(static_cast<int32_t>(raw) - static_cast<int32_t>(regVal));
                break;
            case 0xf:
                raw = clamp12(static_cast<int32_t>(raw) - static_cast<int32_t>(calibVal));
                break;
        }
        this->rawBuf[i] = raw;
    }
}

// ---------------------------------------------------------------------------
// applyBlurAndThreshold()
// ---------------------------------------------------------------------------

void FpgaImagePipeline::applyBlurAndThreshold() {
    const uint32_t W = this->imageWidth;
    const uint32_t H = this->imageHeight;
    const uint32_t k = this->kernelSize;
    const uint32_t half = (k - 1) / 2;
    const uint8_t shift = blurShift(k);

    // Border pixels receive no kernel output — zero both output buffers up front.
    std::ranges::fill(this->blurBuf, uint16_t{0});
    std::ranges::fill(this->threshBuf, uint8_t{0});

    if (W < k || H < k) return;

    // Number of valid kernel positions in each direction.
    const uint32_t numOutCols = W - k + 1;  // left edge c = 0 .. W-k
    const uint32_t numOutRows = H - k + 1;  // top  edge r = 0 .. H-k

    // Running 1-D row sums: rowSums[i] holds the horizontal window sum for the
    // i-th row of the current k-row block.  Size is exactly k (5, 7, or 9).
    std::vector<uint32_t> rowSums(k);

    for (uint32_t rStart = 0; rStart < numOutRows; rStart++) {
        // Initialise each row's sliding sum over columns [0 .. k-1].
        for (uint32_t i = 0; i < k; i++) {
            rowSums[i] = 0;
            for (uint32_t j = 0; j < k; j++) {
                rowSums[i] += this->rawBuf[(rStart + i) * W + j];
            }
        }

        // Slide the 1-D window one column at a time across the valid range.
        for (uint32_t c = 0; c < numOutCols; c++) {
            // Vertical reduction: sum the k row sums → 2-D box sum for this position.
            uint32_t colSum = 0;
            for (uint32_t i = 0; i < k; i++) {
                colSum += rowSums[i];
            }

            // Write the normalised blurred pixel at the centre of the k×k footprint.
            const uint32_t outRow = rStart + half;
            const uint32_t outCol = c + half;
            const uint16_t blurred = static_cast<uint16_t>(colSum >> shift);
            this->blurBuf[outRow * W + outCol] = blurred;

            // Threshold inline — mirrors the FPGA pipeline where each blurred pixel
            // is compared against the threshold immediately as it exits the blur stage.
            if (blurred > this->threshold) {
                const size_t pixIdx = static_cast<size_t>(outRow) * W + outCol;
                const uint32_t byteIdx = static_cast<uint32_t>(pixIdx / 8);
                const uint8_t bitIdx = static_cast<uint8_t>(7u - (pixIdx % 8u));
                this->threshBuf[byteIdx] |= static_cast<uint8_t>(1u << bitIdx);
            }

            // Advance each row's window: add the pixel entering the right edge,
            // subtract the pixel leaving the left edge.
            if (const uint32_t cNext = c + k; cNext < W) {
                for (uint32_t i = 0; i < k; i++) {
                    rowSums[i] += this->rawBuf[(rStart + i) * W + cNext];
                    rowSums[i] -= this->rawBuf[(rStart + i) * W + c];
                }
            }
        }
    }
}

// ---------------------------------------------------------------------------
// computeRowColSums()
// ---------------------------------------------------------------------------

void FpgaImagePipeline::computeRowColSums() {
    std::ranges::fill(this->rowSumBuf, uint16_t{0});
    std::ranges::fill(this->colSumBuf, uint16_t{0});

    const uint32_t W = this->imageWidth;
    const uint32_t H = this->imageHeight;
    for (uint32_t r = 0; r < H; r++) {
        for (uint32_t c = 0; c < W; c++) {
            const size_t idx = static_cast<size_t>(r) * W + c;
            const uint32_t byteIdx = static_cast<uint32_t>(idx / 8);
            const uint8_t bitIdx = static_cast<uint8_t>(7u - (idx % 8u));
            if ((this->threshBuf[byteIdx] >> bitIdx) & 0x01) {
                this->rowSumBuf[r]++;
                this->colSumBuf[c]++;
            }
        }
    }
}

// ---------------------------------------------------------------------------
// computeRoi()
// ---------------------------------------------------------------------------

void FpgaImagePipeline::computeRoi(FpgaBinsMsgPayload& roi) const {
    const uint32_t W = this->imageWidth;
    const uint32_t H = this->imageHeight;
    const uint32_t rSize = this->roiRegionSize;

    const uint32_t numRegionCols = W / rSize;
    const uint32_t numRegionRows = H / rSize;
    const uint32_t numRegions = numRegionCols * numRegionRows;

    std::vector<uint32_t> regionCounts(numRegions, 0);

    for (uint32_t r = 0; r < numRegionRows * rSize; r++) {
        for (uint32_t c = 0; c < numRegionCols * rSize; c++) {
            const size_t idx = static_cast<size_t>(r) * W + c;
            const auto byteIdx = static_cast<uint32_t>(idx / 8);
            const auto bitIdx = static_cast<uint8_t>(7u - (idx % 8u));
            if (((this->threshBuf[byteIdx] >> bitIdx) & 0x01) != 0) {
                const uint32_t regionRow = r / rSize;
                const uint32_t regionCol = c / rSize;
                regionCounts[regionRow * numRegionCols + regionCol]++;
            }
        }
    }

    // Partial sort to find top FPGA_ROI_TOP_COUNT regions by count
    std::vector<uint32_t> indices(numRegions);
    std::iota(indices.begin(), indices.end(), 0);
    const uint32_t topCount = (numRegions < FPGA_ROI_TOP_COUNT) ? numRegions : FPGA_ROI_TOP_COUNT;
    std::ranges::partial_sort(indices, indices.begin() + topCount, [&regionCounts](uint32_t a, uint32_t b) {
        return regionCounts[a] > regionCounts[b];
    });

    roi.regionSize = rSize;
    roi.numValidRegions = topCount;
    for (uint32_t k = 0; k < FPGA_ROI_TOP_COUNT; k++) {
        if (k < topCount) {
            const uint32_t idx = indices[k];
            roi.topBins[k].row = static_cast<uint16_t>(idx / numRegionCols);
            roi.topBins[k].col = static_cast<uint16_t>(idx % numRegionCols);
            roi.topBins[k].count = regionCounts[idx];
        } else {
            roi.topBins[k] = FpgaRoiEntry{};
        }
    }
}

// ---------------------------------------------------------------------------
// publishOutputs()
// ---------------------------------------------------------------------------

void FpgaImagePipeline::publishOutputs(const uint64_t callTime, const uint64_t imageTimeTag, FpgaBinsMsgPayload& roi) {
    const uint32_t W = this->imageWidth;
    const uint32_t H = this->imageHeight;
    const size_t n = static_cast<size_t>(W) * H;

    // Raw image
    FpgaRawImageMsgPayload rawMsg{};
    rawMsg.timeTag = imageTimeTag;
    rawMsg.width = W;
    rawMsg.height = H;
    rawMsg.imagePointer = this->rawBuf.data();
    rawMsg.imageBufferLength = static_cast<int32_t>(n * sizeof(uint16_t));
    rawImageOutMsg.write(&rawMsg, moduleID, callTime);

    // Blurred image
    FpgaRawImageMsgPayload blurMsg{};
    blurMsg.timeTag = imageTimeTag;
    blurMsg.width = W;
    blurMsg.height = H;
    blurMsg.imagePointer = this->blurBuf.data();
    blurMsg.imageBufferLength = static_cast<int32_t>(n * sizeof(uint16_t));
    blurredImageOutMsg.write(&blurMsg, moduleID, callTime);

    // Threshold image
    FpgaThreshImageMsgPayload threshMsg{};
    threshMsg.timeTag = imageTimeTag;
    threshMsg.width = W;
    threshMsg.height = H;
    threshMsg.threshold = this->threshold;
    threshMsg.imagePointer = this->threshBuf.data();
    threshMsg.imageBufferLength = static_cast<int32_t>(this->threshBuf.size());
    threshImageOutMsg.write(&threshMsg, moduleID, callTime);

    // Row/col sums
    FpgaRowColSumMsgPayload rcMsg{};
    rcMsg.timeTag = imageTimeTag;
    rcMsg.numRows = H;
    rcMsg.numCols = W;
    rcMsg.rowSumPointer = this->rowSumBuf.data();
    rcMsg.colSumPointer = this->colSumBuf.data();
    rcMsg.rowSumBufferLength = static_cast<int32_t>(H * sizeof(uint16_t));
    rcMsg.colSumBufferLength = static_cast<int32_t>(W * sizeof(uint16_t));
    rowColSumOutMsg.write(&rcMsg, moduleID, callTime);

    // ROI
    roi.timeTag = imageTimeTag;
    roiOutMsg.write(&roi, moduleID, callTime);

    // Config snapshot
    FpgaPipelineConfigMsgPayload cfgMsg{};
    cfgMsg.timeTag = imageTimeTag;
    cfgMsg.imageWidth = W;
    cfgMsg.imageHeight = H;
    cfgMsg.kernelSize = this->kernelSize;
    cfgMsg.thresholdValue = this->threshold;
    cfgMsg.roiRegionSize = this->roiRegionSize;
    cfgMsg.calibRegA = this->calibRegA;
    cfgMsg.calibRegB = this->calibRegB;
    cfgMsg.calibRegC = this->calibRegC;
    cfgMsg.calibRegD = this->calibRegD;
    cfgMsg.calibEnabled = this->calibEnabled;
    if (!this->calibImageFile.empty()) {
        std::strncpy(cfgMsg.calibImageRef, this->calibImageFile.c_str(), FPGA_CALIB_REF_LEN - 1);
        cfgMsg.calibImageRef[FPGA_CALIB_REF_LEN - 1] = '\0';
    }
    configOutMsg.write(&cfgMsg, moduleID, callTime);
}

// ---------------------------------------------------------------------------
// saveDataToDisk()
// ---------------------------------------------------------------------------

void FpgaImagePipeline::saveDataToDisk(uint64_t timeTagNs, const FpgaBinsMsgPayload& roi) {
    std::string const prefix = this->saveDir + "/" + std::to_string(timeTagNs) + "_";
    uint32_t const W = this->imageWidth;
    uint32_t const H = this->imageHeight;
    size_t const n = static_cast<size_t>(W) * H;

    // raw.tiff — 16-bit grayscale
    cv::Mat const rawMat(static_cast<int>(H), static_cast<int>(W), CV_16UC1, this->rawBuf.data());
    cv::imwrite(prefix + "raw.tiff", rawMat);

    // blurred.tiff — 16-bit grayscale
    cv::Mat blurMat(static_cast<int>(H), static_cast<int>(W), CV_16UC1, this->blurBuf.data());
    cv::imwrite(prefix + "blurred.tiff", blurMat);

    // threshold.png — 8-bit grayscale (unpack 1-bit buffer to 0/255)
    std::vector<uint8_t> threshUnpacked(n);
    for (size_t i = 0; i < n; i++) {
        const uint32_t byteIdx = static_cast<uint32_t>(i / 8);
        const uint8_t bitIdx = static_cast<uint8_t>(7u - (i % 8u));
        threshUnpacked[i] = ((this->threshBuf[byteIdx] >> bitIdx) & 0x01) ? 255 : 0;
    }
    const cv::Mat threshMat(static_cast<int>(H), static_cast<int>(W), CV_8UC1, threshUnpacked.data());
    cv::imwrite(prefix + "threshold.png", threshMat);

    // row_sums.csv
    {
        std::ofstream f(prefix + "row_sums.csv");
        f << "row,count\n";
        for (uint32_t r = 0; r < H; r++) {
            f << r << "," << this->rowSumBuf[r] << "\n";
        }
    }

    // col_sums.csv
    {
        std::ofstream f(prefix + "col_sums.csv");
        f << "col,count\n";
        for (uint32_t c = 0; c < W; c++) {
            f << c << "," << this->colSumBuf[c] << "\n";
        }
    }

    // roi.csv
    {
        std::ofstream f(prefix + "roi.csv");
        f << "rank,region_col,region_row,count\n";
        for (uint32_t k = 0; k < roi.numValidRegions; k++) {
            f << k << "," << roi.topBins[k].col << "," << roi.topBins[k].row << "," << roi.topBins[k].count << "\n";
        }
    }
}
