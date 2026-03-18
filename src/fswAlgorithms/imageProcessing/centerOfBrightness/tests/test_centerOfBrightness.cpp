// SPDX-License-Identifier: ISC
// Copyright (c) 2026, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

#include "../centerOfBrightnessAlgorithm.h"
#include <gtest/gtest.h>
#include <memory>

// Test constants
constexpr int32_t kDefaultBrightnessAvgPoints = 5;
constexpr double kTestTolerance = 1e-6;

// ============================================================================
// MOCK IMAGE READER
// ============================================================================

class MockImageReader : public ImageReaderInterface {
   public:
    // Heap-allocated to avoid stack overflow (~8 MB array)
    std::unique_ptr<std::array<Eigen::Vector2i, kMaxWindowSize>> pixelDataStorage =
        std::make_unique<std::array<Eigen::Vector2i, kMaxWindowSize>>();
    std::array<Eigen::Vector2i, kMaxWindowSize>& pixelData = *pixelDataStorage;

    Eigen::Vector2i getFullImageSize(int32_t cameraId) override { return {100, 100}; }

    int64_t checkForNewImage(int32_t cameraId, int64_t previousImageTimeTag) override { return 1; }

    void getImageAsArray(const Eigen::Vector2i& center,
                         const Eigen::Vector2i& windowSize,
                         std::array<Eigen::Vector2i, kMaxWindowSize>& output) override {
        output = pixelData;
    }
};

// ============================================================================
// FIXTURE CLASS
// ============================================================================

class CenterOfBrightnessAlgorithmTest : public ::testing::Test {
   protected:
    CenterOfBrightnessAlgorithm algorithm{};
    MockImageReader mockReader{};

    void SetUp() override { algorithm.setNumberOfPointsBrightnessAverage(kDefaultBrightnessAvgPoints); }
};

// ============================================================================
// SETTER AND GETTER TESTS
// ============================================================================

TEST_F(CenterOfBrightnessAlgorithmTest, SetAndGetRelativeBrightnessIncreaseThreshold) {
    algorithm.setRelativeBrightnessIncreaseThreshold(0.25);
    EXPECT_NEAR(0.25, algorithm.getRelativeBrightnessIncreaseThreshold(), kTestTolerance);
}

TEST_F(CenterOfBrightnessAlgorithmTest, SetAndGetNumberOfPointsBrightnessAverage) {
    algorithm.setNumberOfPointsBrightnessAverage(10);
    EXPECT_EQ(10, algorithm.getNumberOfPointsBrightnessAverage());
}

// ============================================================================
// EMPTY PIXEL ARRAY (ALL ZEROS) → DEFAULT RESULT
// ============================================================================

TEST_F(CenterOfBrightnessAlgorithmTest, EmptyPixelArrayReturnsDefaultResult) {
    // mockReader.pixelData is all zeros by default (sentinel)
    CobRegionOfInterest roi{};
    roi.center = Eigen::Vector2i(50, 50);
    roi.size = Eigen::Vector2i(100, 100);

    CenterOfBrightnessResult result = algorithm.update(roi, mockReader);

    EXPECT_FALSE(result.valid);
    EXPECT_EQ(0, result.pixelsFound);
    EXPECT_NEAR(0.0, result.centerOfBrightness[0], kTestTolerance);
    EXPECT_NEAR(0.0, result.centerOfBrightness[1], kTestTolerance);
    EXPECT_NEAR(0.0, result.rollingAverageBrightness, kTestTolerance);
    EXPECT_FALSE(result.noPixelTrigger);
    EXPECT_FALSE(result.notExceedingBrightnessIncreaseTrigger);
}

// ============================================================================
// SINGLE NON-ZERO PIXEL → CENTROID AT THAT PIXEL
// ============================================================================

TEST_F(CenterOfBrightnessAlgorithmTest, SingleNonZeroPixelCentroidAtPixel) {
    mockReader.pixelData[0] = Eigen::Vector2i(50, 30);

    CobRegionOfInterest roi{};
    roi.center = Eigen::Vector2i(50, 50);
    roi.size = Eigen::Vector2i(100, 100);

    CenterOfBrightnessResult result = algorithm.update(roi, mockReader);

    EXPECT_TRUE(result.valid);
    EXPECT_EQ(1, result.pixelsFound);
    EXPECT_NEAR(50.0, result.centerOfBrightness[0], kTestTolerance);
    EXPECT_NEAR(30.0, result.centerOfBrightness[1], kTestTolerance);
    EXPECT_FALSE(result.noPixelTrigger);
    EXPECT_FALSE(result.notExceedingBrightnessIncreaseTrigger);
}

// ============================================================================
// SYMMETRIC PIXEL PATTERN → CENTROID AT CENTER
// ============================================================================

TEST_F(CenterOfBrightnessAlgorithmTest, SymmetricPixelPatternCentroidAtCenter) {
    // Four symmetric pixels around (50, 50)
    mockReader.pixelData[0] = Eigen::Vector2i(49, 49);
    mockReader.pixelData[1] = Eigen::Vector2i(51, 49);
    mockReader.pixelData[2] = Eigen::Vector2i(49, 51);
    mockReader.pixelData[3] = Eigen::Vector2i(51, 51);

    CobRegionOfInterest roi{};
    roi.center = Eigen::Vector2i(50, 50);
    roi.size = Eigen::Vector2i(100, 100);

    CenterOfBrightnessResult result = algorithm.update(roi, mockReader);

    EXPECT_TRUE(result.valid);
    EXPECT_EQ(4, result.pixelsFound);
    EXPECT_NEAR(50.0, result.centerOfBrightness[0], kTestTolerance);
    EXPECT_NEAR(50.0, result.centerOfBrightness[1], kTestTolerance);
}

// ============================================================================
// BRIGHTNESS THRESHOLD VALIDATION
// ============================================================================

TEST_F(CenterOfBrightnessAlgorithmTest, BrightnessIncreaseThresholdInvalidatesResult) {
    // Set a high brightness increase threshold
    algorithm.setRelativeBrightnessIncreaseThreshold(0.5);
    algorithm.setNumberOfPointsBrightnessAverage(2);

    mockReader.pixelData[0] = Eigen::Vector2i(50, 50);

    CobRegionOfInterest roi{};
    roi.center = Eigen::Vector2i(50, 50);
    roi.size = Eigen::Vector2i(100, 100);

    // First call: no prior history, brightnessIncrease = 0.0 < 0.5 threshold → not valid
    CenterOfBrightnessResult result1 = algorithm.update(roi, mockReader);
    EXPECT_FALSE(result1.valid);
    EXPECT_FALSE(result1.noPixelTrigger);  // pixels were found
    EXPECT_FALSE(result1.notExceedingBrightnessIncreaseTrigger);

    // Second call with same data: increase = 0.0 < 0.5 threshold, still not valid
    CenterOfBrightnessResult result2 = algorithm.update(roi, mockReader);
    EXPECT_FALSE(result2.valid);
    EXPECT_FALSE(result2.notExceedingBrightnessIncreaseTrigger);
}

TEST_F(CenterOfBrightnessAlgorithmTest, BrightnessIncreaseAboveThresholdValidatesResult) {
    algorithm.setRelativeBrightnessIncreaseThreshold(0.5);
    algorithm.setNumberOfPointsBrightnessAverage(2);

    mockReader.pixelData[0] = Eigen::Vector2i(50, 50);

    CobRegionOfInterest roi{};
    roi.center = Eigen::Vector2i(50, 50);
    roi.size = Eigen::Vector2i(100, 100);

    // First call: establishes brightness history with 1 pixel (avgOld = 0 → increase = 0)
    CenterOfBrightnessResult result1 = algorithm.update(roi, mockReader);
    EXPECT_FALSE(result1.valid);

    // Second call with 4 pixels: avgOld = 1.0, history becomes [4, 1], avgNew = 2.5
    // increase = (2.5 - 1.0) / 1.0 = 1.5 >= 0.5 threshold → valid
    mockReader.pixelData[1] = Eigen::Vector2i(51, 50);
    mockReader.pixelData[2] = Eigen::Vector2i(50, 51);
    mockReader.pixelData[3] = Eigen::Vector2i(51, 51);

    CenterOfBrightnessResult result2 = algorithm.update(roi, mockReader);
    EXPECT_TRUE(result2.valid);
    EXPECT_FALSE(result2.notExceedingBrightnessIncreaseTrigger);
    EXPECT_EQ(4, result2.pixelsFound);
}

// ============================================================================
// ROLLING AVERAGE BRIGHTNESS TRACKING
// ============================================================================

TEST_F(CenterOfBrightnessAlgorithmTest, RollingAverageBrightnessTracking) {
    algorithm.setNumberOfPointsBrightnessAverage(3);

    mockReader.pixelData[0] = Eigen::Vector2i(50, 50);

    CobRegionOfInterest roi{};
    roi.center = Eigen::Vector2i(50, 50);
    roi.size = Eigen::Vector2i(100, 100);

    // First update establishes initial brightness
    CenterOfBrightnessResult result1 = algorithm.update(roi, mockReader);
    EXPECT_GT(result1.rollingAverageBrightness, 0.0);
    double firstBrightness = result1.rollingAverageBrightness;

    // Second update with same data should give similar brightness
    CenterOfBrightnessResult result2 = algorithm.update(roi, mockReader);
    EXPECT_NEAR(firstBrightness, result2.rollingAverageBrightness, kTestTolerance);
}

// ============================================================================
// RESET CLEARS BRIGHTNESS HISTORY
// ============================================================================

TEST_F(CenterOfBrightnessAlgorithmTest, ResetClearsBrightnessHistory) {
    mockReader.pixelData[0] = Eigen::Vector2i(50, 50);

    CobRegionOfInterest roi{};
    roi.center = Eigen::Vector2i(50, 50);
    roi.size = Eigen::Vector2i(100, 100);

    // Build up some brightness history
    algorithm.update(roi, mockReader);
    algorithm.update(roi, mockReader);

    // Reset should clear it
    algorithm.reset();

    // After reset, the first call should behave like a fresh start
    // (no prior brightness history, so averageBrightnessOld = 0)
    CenterOfBrightnessResult result = algorithm.update(roi, mockReader);
    EXPECT_TRUE(result.valid);
    EXPECT_FALSE(result.noPixelTrigger);
}
