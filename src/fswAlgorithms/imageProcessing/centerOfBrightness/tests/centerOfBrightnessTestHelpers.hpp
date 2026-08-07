// SPDX-License-Identifier: ISC
// Copyright (c) 2026, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

#ifndef CENTER_OF_BRIGHTNESS_TEST_HELPERS_HPP
#define CENTER_OF_BRIGHTNESS_TEST_HELPERS_HPP

#include "../centerOfBrightnessAlgorithm.h"
#include <Eigen/Core>
#include <algorithm>
#include <cmath>
#include <cstddef>
#include <deque>
#include <gtest/gtest.h>
#include <vector>

//! [pix] Width and height of the image that the fake reader gives
constexpr int32_t kFuzzImageSize = 4'096;

/*! Select the pixels that are inside the region of interest window.
 @return the pixels that are inside the window
 @param pixels Candidate pixel coordinates
 @param center Window center
 @param windowSize Window width and height
 @param imageSize Full image dimensions, which limit the window
 */
inline std::vector<Eigen::Vector2i> windowedPixels(
    std::vector<Eigen::Vector2i> const &pixels,
    Eigen::Vector2i const &center,
    Eigen::Vector2i const &windowSize,
    Eigen::Vector2i const &imageSize
) {
    int32_t const left = std::max(0, center[0] - windowSize[0] / 2);
    int32_t const top = std::max(0, center[1] - windowSize[1] / 2);
    int32_t const right = std::min(imageSize[0] - 1, center[0] + windowSize[0] / 2);
    int32_t const bottom = std::min(imageSize[1] - 1, center[1] + windowSize[1] / 2);

    std::vector<Eigen::Vector2i> inside;
    inside.reserve(pixels.size());
    for (auto const &pixel : pixels) {
        if (pixel[0] >= left && pixel[0] <= right && pixel[1] >= top && pixel[1] <= bottom) { inside.push_back(pixel); }
    }
    return inside;
}

// ============================================================================
// FUZZ IMAGE READER
// ============================================================================

class FuzzImageReader : public ImageReaderInterface {
   public:
    std::vector<Eigen::Vector2i> pixels;  //!< what the fake camera sees, before windowing

    Eigen::Vector2i getFullImageSize(int32_t /*cameraId*/) override {
        return {kFuzzImageSize, kFuzzImageSize};
    }

    int64_t getCurrentImageTimeTag(int32_t /*cameraId*/, int64_t /*previousImageTimeTag*/) override { return 1; }

    void getImageAsArray(
        Eigen::Vector2i const &center,
        Eigen::Vector2i const &windowSize,
        std::array<Eigen::Vector2i, kMaxWindowSize> &output
    ) override {
        std::vector<Eigen::Vector2i> const inside =
            windowedPixels(this->pixels, center, windowSize, this->getFullImageSize(0));
        size_t const writeCount = std::min(inside.size(), static_cast<size_t>(kMaxWindowSize));

        output.fill(Eigen::Vector2i::Zero());
        for (size_t i = 0; i < writeCount; ++i) {
            output[i] = inside[i];
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
    pixels.reserve(static_cast<size_t>(numPixels));
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

    // Run the reference with the same pixels that the reader gave to the algorithm
    std::vector<Eigen::Vector2i> const visible =
        windowedPixels(pixels, roi.center, roi.size, reader.getFullImageSize(0));
    ReferenceState refState;
    refState.maxHistorySize = avgWindowSize;
    CenterOfBrightnessResult refResult = referenceUpdate(visible, brightnessThreshold, refState);

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
    EXPECT_LE(result.pixelsFound, static_cast<int32_t>(visible.size()));
    if (visible.empty()) { EXPECT_EQ(result.pixelsFound, 0); }
    if (result.valid && result.pixelsFound > 0) {
        // The centroid must stay inside the bounding box of the pixels that supplied it.
        // The algorithm uses (0,0) as its "no pixel" sentinel. Thus a (0,0) pixel supplies
        // nothing, and it must not increase the size of that box.
        bool haveBounds = false;
        int32_t minX = 0;
        int32_t maxX = 0;
        int32_t minY = 0;
        int32_t maxY = 0;
        for (auto const &pixel : visible) {
            if (pixel.isZero()) { continue; }
            if (!haveBounds) {
                minX = maxX = pixel[0];
                minY = maxY = pixel[1];
                haveBounds = true;
                continue;
            }
            minX = std::min(minX, pixel[0]);
            maxX = std::max(maxX, pixel[0]);
            minY = std::min(minY, pixel[1]);
            maxY = std::max(maxY, pixel[1]);
        }
        ASSERT_TRUE(haveBounds);
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
        // The coordinates are deterministic and only the count changes. They are relative
        // to the window center, thus the window keeps all of them. This test examines the
        // rolling average, and pixels outside the window would give zero at each step.
        std::vector<Eigen::Vector2i> pixels;
        pixels.reserve(static_cast<size_t>(pixelCount));
        for (int32_t i = 0; i < pixelCount; ++i) {
            pixels.emplace_back(roi.center[0] - 25 + (i % 50), roi.center[1] - 25 + (i / 50));
        }
        reader.pixels = pixels;

        CenterOfBrightnessResult result = alg.update(roi, reader);
        std::vector<Eigen::Vector2i> const visible =
            windowedPixels(pixels, roi.center, roi.size, reader.getFullImageSize(0));
        ASSERT_EQ(visible.size(), pixels.size());
        CenterOfBrightnessResult refResult = referenceUpdate(visible, brightnessThreshold, refState);

        EXPECT_EQ(result.pixelsFound, refResult.pixelsFound);
        EXPECT_NEAR(result.rollingAverageBrightness, refResult.rollingAverageBrightness, 1e-9);
        EXPECT_EQ(result.valid, refResult.valid);
        EXPECT_EQ(result.noPixelTrigger, refResult.noPixelTrigger);
        EXPECT_EQ(result.notExceedingBrightnessIncreaseTrigger, refResult.notExceedingBrightnessIncreaseTrigger);
        EXPECT_TRUE(std::isfinite(result.rollingAverageBrightness));
    }
}

#endif  // CENTER_OF_BRIGHTNESS_TEST_HELPERS_HPP
