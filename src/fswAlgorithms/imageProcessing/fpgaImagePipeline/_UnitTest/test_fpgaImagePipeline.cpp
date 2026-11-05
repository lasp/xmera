// SPDX-License-Identifier: ISC
// Copyright (c) 2026, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

#include <architecture/messaging/messaging.h>
#include <architecture/msgPayloadDef/CameraImageMsgPayload.h>

#include <fswAlgorithms/imageProcessing/fpgaImagePipeline/fpgaImagePipeline.h>
#include <gtest/gtest.h>

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <numeric>
#include <stdexcept>
#include <vector>

namespace {
    //! Pipeline settings for one test image.
    struct PipelineConfig {
        uint32_t width;
        uint32_t height;
        uint8_t kernelSize;
        uint16_t threshold;
        uint32_t roiRegionSize;
    };

    //! This harness connects the pipeline to a raw camera image message and to readers of its outputs.
    class PipelineHarness {
    public:
        PipelineHarness() {
            this->pipeline.imageInMsg.subscribeTo(&this->imageInMsg);
            this->rowColSumOut = this->pipeline.rowColSumOutMsg.addSubscriber();
            this->roiOut = this->pipeline.roiOutMsg.addSubscriber();
            this->binGridOut = this->pipeline.binGridOutMsg.addSubscriber();
        }

        PipelineHarness(PipelineHarness const &) = delete;
        PipelineHarness &operator=(PipelineHarness const &) = delete;

        //! Configures and resets the pipeline, sends the raw 12-bit image, and runs one update.
        void run(PipelineConfig const &config, std::vector<uint16_t> &image) {
            this->pipeline.setImageWidth(config.width);
            this->pipeline.setImageHeight(config.height);
            this->pipeline.setKernelSize(config.kernelSize);
            this->pipeline.setThreshold(config.threshold);
            this->pipeline.setRoiRegionSize(config.roiRegionSize);
            this->pipeline.reset(0);

            CameraImageMsgPayload payload{};
            payload.valid = 1;
            payload.imagePointer = image.data();
            payload.imageBufferLength = static_cast<int32_t>(image.size() * sizeof(uint16_t));
            this->imageInMsg.write(payload, 0, 0);
            this->pipeline.updateState(0);
        }

        FpgaImagePipeline pipeline;                         //!< pipeline that the tests examine
        Message<CameraImageMsgPayload> imageInMsg;          //!< input camera image message
        ReadFunctor<FpgaRowColSumMsgPayload> rowColSumOut;  //!< reader of the row/col sum message
        ReadFunctor<FpgaBinsMsgPayload> roiOut;             //!< reader of the top-8 ROI message
        ReadFunctor<FpgaBinGridMsgPayload> binGridOut;      //!< reader of the ROI bin grid message
    };

    //! Makes a width x height image with all pixels set to value.
    std::vector<uint16_t> uniformImage(uint32_t width, uint32_t height, uint16_t value) {
        return std::vector<uint16_t>(static_cast<size_t>(width) * height, value);
    }

    //! Checks the message sizes and that counts and intensities agree across rows, columns and the bin grid.
    void expectConsistentOutputs(PipelineHarness &harness, PipelineConfig const &config) {
        FpgaRowColSumMsgPayload const rc = harness.rowColSumOut();
        ASSERT_EQ(rc.numRows, config.height);
        ASSERT_EQ(rc.numCols, config.width);

        uint32_t const expectedRegionRows = (config.height + config.roiRegionSize - 1) / config.roiRegionSize;
        uint32_t const expectedRegionCols = (config.width + config.roiRegionSize - 1) / config.roiRegionSize;
        FpgaBinGridMsgPayload const grid = harness.binGridOut();
        ASSERT_EQ(grid.regionSize, config.roiRegionSize);
        ASSERT_EQ(grid.numRegionRows, expectedRegionRows);
        ASSERT_EQ(grid.numRegionCols, expectedRegionCols);
        size_t const numBins = static_cast<size_t>(expectedRegionRows) * expectedRegionCols;
        ASSERT_EQ(grid.numRegions, numBins);

        uint64_t const rowCountTotal = std::accumulate(rc.rowSums, rc.rowSums + rc.numRows, uint64_t{0});
        uint64_t const colCountTotal = std::accumulate(rc.colSums, rc.colSums + rc.numCols, uint64_t{0});
        uint64_t binCountTotal = 0;
        for (size_t i = 0; i < numBins; ++i) { binCountTotal += grid.regions[i].litPixelCount; }
        EXPECT_EQ(rowCountTotal, colCountTotal);
        EXPECT_EQ(rowCountTotal, binCountTotal);

        uint64_t const rowIntensityTotal =
            std::accumulate(rc.rowIntensities, rc.rowIntensities + rc.numRows, uint64_t{0});
        uint64_t const colIntensityTotal =
            std::accumulate(rc.colIntensities, rc.colIntensities + rc.numCols, uint64_t{0});
        EXPECT_EQ(rowIntensityTotal, colIntensityTotal);

        uint32_t const size = config.roiRegionSize;
        for (uint32_t r = 0; r < grid.numRegionRows; ++r) {
            for (uint32_t c = 0; c < grid.numRegionCols; ++c) {
                FpgaBinRegion const &region = grid.regions[r * grid.numRegionCols + c];
                uint32_t const width = std::min(size, config.width - region.x);
                uint32_t const height = std::min(size, config.height - region.y);
                EXPECT_EQ(region.x, c * size) << "region (" << r << ", " << c << ")";
                EXPECT_EQ(region.y, r * size) << "region (" << r << ", " << c << ")";
                EXPECT_LE(region.litPixelCount, width * height) << "region (" << r << ", " << c << ")";
                EXPECT_EQ(harness.pipeline.getBinCount(r, c), region.litPixelCount);
            }
        }
    }
}  // namespace

//! A uniform 16x8 image of value 500 with kernel 5 gives a blurred interior of 25*500 >> 1 = 6250.
//! A threshold of 6249 sets the interior rows 2-5 and columns 2-13. Each row then sums 12 * 6250
//! and each column sums 4 * 6250.
TEST(FpgaImagePipeline, intensitySumsOfUniformInterior) {
    PipelineConfig const config{16, 8, 5, 6'249, 64};
    std::vector<uint16_t> image = uniformImage(config.width, config.height, 500);
    PipelineHarness harness;
    harness.run(config, image);

    for (uint32_t r = 0; r < config.height; ++r) {
        uint32_t const expected = (r >= 2 && r <= 5) ? 12u * 6'250u : 0u;
        EXPECT_EQ(harness.pipeline.getRowIntensity(r), expected) << "row " << r;
    }
    for (uint32_t c = 0; c < config.width; ++c) {
        uint32_t const expected = (c >= 2 && c <= 13) ? 4u * 6'250u : 0u;
        EXPECT_EQ(harness.pipeline.getColIntensity(c), expected) << "col " << c;
    }
    expectConsistentOutputs(harness, config);
}

//! The image is smaller than one region. Thus the grid has one partial region that holds all 48 set pixels.
TEST(FpgaImagePipeline, imageSmallerThanRegionGivesOnePartialBin) {
    PipelineConfig const config{16, 8, 5, 6'249, 64};
    std::vector<uint16_t> image = uniformImage(config.width, config.height, 500);
    PipelineHarness harness;
    harness.run(config, image);

    EXPECT_EQ(harness.pipeline.getNumRegionRows(), 1u);
    EXPECT_EQ(harness.pipeline.getNumRegionCols(), 1u);
    EXPECT_EQ(harness.pipeline.getBinCount(0, 0), 48u);

    FpgaBinsMsgPayload const roi = harness.roiOut();
    EXPECT_EQ(roi.numValidRegions, 1u);
    EXPECT_EQ(roi.topBins[0].row, 0u);
    EXPECT_EQ(roi.topBins[0].col, 0u);
    EXPECT_EQ(roi.topBins[0].count, 48u);
}

//! When no blurred pixel is above the threshold, all counts, intensities and bins are zero.
TEST(FpgaImagePipeline, thresholdAboveAllPixelsGivesZeroSums) {
    PipelineConfig const config{16, 8, 5, std::numeric_limits<uint16_t>::max(), 64};
    std::vector<uint16_t> image = uniformImage(config.width, config.height, 500);
    PipelineHarness harness;
    harness.run(config, image);

    for (uint32_t r = 0; r < config.height; ++r) { EXPECT_EQ(harness.pipeline.getRowIntensity(r), 0u); }
    for (uint32_t c = 0; c < config.width; ++c) { EXPECT_EQ(harness.pipeline.getColIntensity(c), 0u); }
    EXPECT_EQ(harness.pipeline.getBinCount(0, 0), 0u);
    expectConsistentOutputs(harness, config);
}

//! A 4096-wide image of full-scale pixels gives a blurred value of 25*4095 >> 1 = 51187 in the 4092
//! interior columns of row 2. The row sum 4092 * 51187 is much larger than a 16-bit value can hold.
TEST(FpgaImagePipeline, fullScaleWideRowSumKeepsAllBits) {
    PipelineConfig const config{4'096, 5, 5, 0, 64};
    std::vector<uint16_t> image = uniformImage(config.width, config.height, 4'095);
    PipelineHarness harness;
    harness.run(config, image);

    EXPECT_EQ(harness.pipeline.getRowIntensity(2), 4'092u * 51'187u);
    EXPECT_EQ(harness.pipeline.getColIntensity(2), 51'187u);
    EXPECT_EQ(harness.pipeline.getColIntensity(0), 0u);
    expectConsistentOutputs(harness, config);
}

//! A 200x150 image with region size 64 has a 3x4 grid. The last row and column of regions are partial.
//! A bright 6x6 block at rows 135-140 and columns 192-197 sets the blurred pixels at rows 133-142 and
//! columns 190-197. Region (2, 3) holds 10 * 6 = 60 of them and region (2, 2) holds 10 * 2 = 20.
TEST(FpgaImagePipeline, partialEdgeRegionIsBinnedAndRanked) {
    PipelineConfig const config{200, 150, 5, 100, 64};
    std::vector<uint16_t> image = uniformImage(config.width, config.height, 0);
    for (uint32_t r = 135; r <= 140; ++r) {
        for (uint32_t c = 192; c <= 197; ++c) { image[static_cast<size_t>(r) * config.width + c] = 4'095; }
    }
    PipelineHarness harness;
    harness.run(config, image);

    ASSERT_EQ(harness.pipeline.getNumRegionRows(), 3u);
    ASSERT_EQ(harness.pipeline.getNumRegionCols(), 4u);
    for (uint32_t r = 0; r < 3; ++r) {
        for (uint32_t c = 0; c < 4; ++c) {
            uint32_t expected = 0;
            if (r == 2 && c == 3) { expected = 60; }
            if (r == 2 && c == 2) { expected = 20; }
            EXPECT_EQ(harness.pipeline.getBinCount(r, c), expected) << "region (" << r << ", " << c << ")";
        }
    }

    FpgaBinsMsgPayload const roi = harness.roiOut();
    EXPECT_EQ(roi.numValidRegions, 8u);
    EXPECT_EQ(roi.topBins[0].row, 2u);
    EXPECT_EQ(roi.topBins[0].col, 3u);
    EXPECT_EQ(roi.topBins[0].count, 60u);
    EXPECT_EQ(roi.topBins[1].row, 2u);
    EXPECT_EQ(roi.topBins[1].col, 2u);
    EXPECT_EQ(roi.topBins[1].count, 20u);
    expectConsistentOutputs(harness, config);
}

//! An image larger than the fixed row/col message arrays is rejected at reset.
TEST(FpgaImagePipeline, imageLargerThanMessageCapacityIsRejected) {
    FpgaImagePipeline wide;
    wide.setImageWidth(FPGA_ROW_COL_MAX_COLS + 1);
    EXPECT_THROW(wide.reset(0), std::invalid_argument);

    FpgaImagePipeline tall;
    tall.setImageHeight(FPGA_ROW_COL_MAX_ROWS + 1);
    EXPECT_THROW(tall.reset(0), std::invalid_argument);
}
