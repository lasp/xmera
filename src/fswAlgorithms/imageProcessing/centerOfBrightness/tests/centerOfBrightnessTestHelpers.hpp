// SPDX-License-Identifier: ISC
// Copyright (c) 2026, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

#ifndef CENTER_OF_BRIGHTNESS_TEST_HELPERS_HPP
#define CENTER_OF_BRIGHTNESS_TEST_HELPERS_HPP

#include "../centerOfBrightnessAlgorithm.h"
#include <Eigen/Core>
#include <cmath>
#include <deque>
#include <gtest/gtest.h>
#include <vector>

// ============================================================================
// FUZZ IMAGE READER
// ============================================================================

class FuzzImageReader : public ImageReaderInterface {
   public:
    std::vector<Eigen::Vector2i> pixels;

    Eigen::Vector2i getFullImageSize(int32_t /*cameraId*/) override { return {4096, 4096}; }

    int64_t getCurrentImageTimeTag(int32_t /*cameraId*/, int64_t /*previousImageTimeTag*/) override { return 1; }

    void getImageAsArray(const Eigen::Vector2i& /*center*/,
                         const Eigen::Vector2i& /*windowSize*/,
                         std::array<Eigen::Vector2i, kMaxWindowSize>& output) override {
        output.fill(Eigen::Vector2i::Zero());
        for (size_t i = 0; i < pixels.size() && i < kMaxWindowSize; ++i) {
            output[i] = pixels[i];
        }
    }
};

// ============================================================================
// REFERENCE STATE (brightness history for multi-step testing)
// ============================================================================

struct ReferenceState {
    std::deque<double> brightnessHistory;
    int32_t maxHistorySize = 1;
};

// ============================================================================
// REFERENCE IMPLEMENTATION
// ============================================================================

inline CenterOfBrightnessResult referenceUpdate(const std::vector<Eigen::Vector2i>& pixels,
                                                double brightnessThreshold,
                                                ReferenceState& state) {
    // Compute centroid of non-zero pixels
    Eigen::Vector2d centroid = Eigen::Vector2d::Zero();
    int32_t count = 0;
    for (const auto& p : pixels) {
        if (p.isZero()) {
            continue;
        }
        centroid[0] += p[0];
        centroid[1] += p[1];
        ++count;
    }
    if (count > 0) {
        centroid /= count;
    }

    // Build result (matching defaults in CenterOfBrightnessResult)
    CenterOfBrightnessResult result{};

    if (count > 0) {
        // Compute old brightness average
        double avgOld = 0.0;
        if (!state.brightnessHistory.empty()) {
            double sum = 0.0;
            for (double v : state.brightnessHistory) {
                sum += v;
            }
            avgOld = sum / static_cast<double>(state.brightnessHistory.size());
        }

        // Update history: grow if not at full size, then shift and insert at front
        if (static_cast<int32_t>(state.brightnessHistory.size()) < state.maxHistorySize) {
            state.brightnessHistory.push_back(0.0);
        }
        for (auto i = static_cast<int>(state.brightnessHistory.size()) - 1; i > 0; --i) {
            state.brightnessHistory[static_cast<size_t>(i)] = state.brightnessHistory[static_cast<size_t>(i - 1)];
        }
        state.brightnessHistory[0] = static_cast<double>(count);

        // Compute new brightness average
        double sumNew = 0.0;
        for (double v : state.brightnessHistory) {
            sumNew += v;
        }
        double avgNew = sumNew / static_cast<double>(state.brightnessHistory.size());

        // Compute relative increase
        double brightnessIncrease = 0.0;
        if (avgOld > 0.0) {
            brightnessIncrease = (avgNew - avgOld) / avgOld;
        }

        result.noPixelTrigger = false;
        if (brightnessIncrease >= brightnessThreshold) {
            result.valid = true;
            result.centerOfBrightness = centroid;
            result.pixelsFound = count;
            result.notExceedingBrightnessIncreaseTrigger = false;
        }
        result.rollingAverageBrightness = avgNew;
    }

    return result;
}

// ============================================================================
// FUZZ TEST: single step
// ============================================================================

inline void fuzzCenterOfBrightness(int32_t roiCenterX,
                                   int32_t roiCenterY,
                                   int32_t roiSizeW,
                                   int32_t roiSizeH,
                                   int32_t numPixels,
                                   std::vector<int32_t> pixelXs,
                                   std::vector<int32_t> pixelYs,
                                   double brightnessThreshold,
                                   int32_t avgWindowSize) {
    numPixels = std::min(numPixels, static_cast<int32_t>(pixelXs.size()));
    numPixels = std::min(numPixels, static_cast<int32_t>(pixelYs.size()));

    // Build pixel vector
    std::vector<Eigen::Vector2i> pixels;
    pixels.reserve(numPixels);
    for (int32_t i = 0; i < numPixels; ++i) {
        pixels.emplace_back(pixelXs[static_cast<size_t>(i)], pixelYs[static_cast<size_t>(i)]);
    }

    // Set up algorithm
    CenterOfBrightnessAlgorithm alg;
    alg.setRelativeBrightnessIncreaseThreshold(brightnessThreshold);
    alg.setNumberOfPointsBrightnessAverage(avgWindowSize);

    // Set up fake image reader
    FuzzImageReader reader;
    reader.pixels = pixels;

    // Set up ROI
    CobRegionOfInterest roi;
    roi.center = Eigen::Vector2i(roiCenterX, roiCenterY);
    roi.size = Eigen::Vector2i(roiSizeW, roiSizeH);

    // Run algorithm
    CenterOfBrightnessResult result = alg.update(roi, reader);

    // Run reference
    ReferenceState refState;
    refState.maxHistorySize = avgWindowSize;
    CenterOfBrightnessResult refResult = referenceUpdate(pixels, brightnessThreshold, refState);

    // Reference correctness
    EXPECT_NEAR(result.centerOfBrightness[0], refResult.centerOfBrightness[0], 1e-9);
    EXPECT_NEAR(result.centerOfBrightness[1], refResult.centerOfBrightness[1], 1e-9);
    EXPECT_EQ(result.pixelsFound, refResult.pixelsFound);
    EXPECT_NEAR(result.rollingAverageBrightness, refResult.rollingAverageBrightness, 1e-9);
    EXPECT_EQ(result.valid, refResult.valid);
    EXPECT_EQ(result.noPixelTrigger, refResult.noPixelTrigger);
    EXPECT_EQ(result.notExceedingBrightnessIncreaseTrigger, refResult.notExceedingBrightnessIncreaseTrigger);

    // Finiteness
    EXPECT_TRUE(std::isfinite(result.centerOfBrightness[0]));
    EXPECT_TRUE(std::isfinite(result.centerOfBrightness[1]));
    EXPECT_TRUE(std::isfinite(result.rollingAverageBrightness));

    // Structural invariants
    EXPECT_GE(result.pixelsFound, 0);
    if (numPixels == 0) {
        EXPECT_FALSE(result.noPixelTrigger);
        EXPECT_EQ(result.pixelsFound, 0);
    }
    if (result.valid && result.pixelsFound > 0) {
        int32_t minX = pixelXs[0];
        int32_t maxX = pixelXs[0];
        int32_t minY = pixelYs[0];
        int32_t maxY = pixelYs[0];
        for (int32_t i = 0; i < numPixels; ++i) {
            minX = std::min(minX, pixelXs[static_cast<size_t>(i)]);
            maxX = std::max(maxX, pixelXs[static_cast<size_t>(i)]);
            minY = std::min(minY, pixelYs[static_cast<size_t>(i)]);
            maxY = std::max(maxY, pixelYs[static_cast<size_t>(i)]);
        }
        EXPECT_GE(result.centerOfBrightness[0], static_cast<double>(minX) - 1e-9);
        EXPECT_LE(result.centerOfBrightness[0], static_cast<double>(maxX) + 1e-9);
        EXPECT_GE(result.centerOfBrightness[1], static_cast<double>(minY) - 1e-9);
        EXPECT_LE(result.centerOfBrightness[1], static_cast<double>(maxY) + 1e-9);
    }
}

// ============================================================================
// FUZZ TEST: multi-step (rolling average statefulness)
// ============================================================================

inline void fuzzMultiStepBrightness(int32_t avgWindowSize,
                                    double brightnessThreshold,
                                    std::vector<int32_t> pixelCountsPerStep) {
    CenterOfBrightnessAlgorithm alg;
    alg.setRelativeBrightnessIncreaseThreshold(brightnessThreshold);
    alg.setNumberOfPointsBrightnessAverage(avgWindowSize);

    ReferenceState refState;
    refState.maxHistorySize = avgWindowSize;

    FuzzImageReader reader;
    CobRegionOfInterest roi;
    roi.center = Eigen::Vector2i(500, 500);
    roi.size = Eigen::Vector2i(100, 100);

    for (int32_t pixelCount : pixelCountsPerStep) {
        // Build pixel vector with deterministic coordinates but varying count
        std::vector<Eigen::Vector2i> pixels;
        for (int32_t i = 0; i < pixelCount; ++i) {
            pixels.emplace_back(100 + (i % 50), 200 + (i / 50));
        }
        reader.pixels = pixels;

        CenterOfBrightnessResult result = alg.update(roi, reader);
        CenterOfBrightnessResult refResult = referenceUpdate(pixels, brightnessThreshold, refState);

        EXPECT_EQ(result.pixelsFound, refResult.pixelsFound);
        EXPECT_NEAR(result.rollingAverageBrightness, refResult.rollingAverageBrightness, 1e-9);
        EXPECT_EQ(result.valid, refResult.valid);
        EXPECT_EQ(result.noPixelTrigger, refResult.noPixelTrigger);
        EXPECT_EQ(result.notExceedingBrightnessIncreaseTrigger, refResult.notExceedingBrightnessIncreaseTrigger);
        EXPECT_TRUE(std::isfinite(result.rollingAverageBrightness));
    }
}

#endif  // CENTER_OF_BRIGHTNESS_TEST_HELPERS_HPP
