// SPDX-License-Identifier: ISC
// Copyright (c) 2026, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

#include "../regionsOfInterestAlgorithm.h"
#include <gtest/gtest.h>
#include <cmath>
#include <array>

// Test constants
constexpr int32_t DEFAULT_IMAGE_WIDTH = 1024;
constexpr int32_t DEFAULT_IMAGE_HEIGHT = 768;
constexpr int32_t DEFAULT_MAX_SEPARATION = 1000;
constexpr int32_t DEFAULT_MIN_DETECTION = 2;
constexpr double TEST_TOLERANCE = 1e-6;

/*! @brief Test and validate RegionsOfInterestAlgorithm region identification
 *
 *  This function sets up the algorithm with test regions and validates that
 *  the identified ROI matches expected behavior.
 *
 *  @param regions Input array of detected regions
 *  @param expectedCenter Expected center of the identified ROI
 *  @param expectedPixels Expected pixel count of the identified ROI
 *  @param windowCenter Optional window center (if not provided, no windowing)
 *  @param windowWidth Optional window width
 *  @param windowHeight Optional window height
 *  @param maxSeparation Maximum separation for region merging
 */
inline void testRegionIdentification(const std::array<RegionOfInterest, MAX_NUMBER_REGIONS>& regions,
                                     const Eigen::Vector2i& expectedCenter,
                                     int expectedPixels,
                                     const Eigen::Vector2i& windowCenter = Eigen::Vector2i::Zero(),
                                     int32_t windowWidth = 0,
                                     int32_t windowHeight = 0,
                                     int32_t maxSeparation = DEFAULT_MAX_SEPARATION) {
    RegionsOfInterestAlgorithm algorithm{};

    if (!windowCenter.isZero() && windowWidth > 0 && windowHeight > 0) {
        algorithm.setWindowCenter(windowCenter);
        algorithm.setWindowSize(windowWidth, windowHeight);
    }

    algorithm.setMaxRoiSeparation(maxSeparation);
    algorithm.reset();

    RegionOfInterest result = algorithm.update(regions);

    EXPECT_EQ(expectedCenter[0], result.centerOfBrightness[0]);
    EXPECT_EQ(expectedCenter[1], result.centerOfBrightness[1]);
    EXPECT_EQ(expectedPixels, result.numberOfPixels);
}

// ============================================================================
// FIXTURE CLASS
// ============================================================================

class RegionsOfInterestAlgorithmTest : public ::testing::Test {
   protected:
    RegionsOfInterestAlgorithm algorithm{};

    void SetUp() override {
        // Common setup for tests
        algorithm.setMaxRoiSeparation(DEFAULT_MAX_SEPARATION);
        algorithm.setMinimumDetectionSize(DEFAULT_MIN_DETECTION);
    }
};

// ============================================================================
// SETTER AND GETTER TESTS
// ============================================================================

TEST_F(RegionsOfInterestAlgorithmTest, SetAndGetMaxRoiSeparation) {
    constexpr int32_t testSeparation = 500;
    algorithm.setMaxRoiSeparation(testSeparation);
    EXPECT_EQ(testSeparation, algorithm.getMaxRoiSeparation());
}

TEST_F(RegionsOfInterestAlgorithmTest, SetAndGetCameraId) {
    constexpr int32_t testId = 42;
    algorithm.setCameraId(testId);
    EXPECT_EQ(testId, algorithm.getCameraId());
}

TEST_F(RegionsOfInterestAlgorithmTest, SetAndGetMinimumDetectionSize) {
    constexpr int32_t testMinSize = 10;
    algorithm.setMinimumDetectionSize(testMinSize);
    EXPECT_EQ(testMinSize, algorithm.getMinimumDetectionSize());
}

TEST_F(RegionsOfInterestAlgorithmTest, SetAndGetWindowCenter) {
    const Eigen::Vector2i testCenter(512, 384);
    algorithm.setWindowCenter(testCenter);
    Eigen::Vector2i result = algorithm.getWindowCenter();
    EXPECT_EQ(testCenter[0], result[0]);
    EXPECT_EQ(testCenter[1], result[1]);
}

TEST_F(RegionsOfInterestAlgorithmTest, SetAndGetWindowSize) {
    constexpr int32_t testWidth = 640;
    constexpr int32_t testHeight = 480;
    algorithm.setWindowSize(testWidth, testHeight);
    Eigen::Vector2i result = algorithm.getWindowSize();
    EXPECT_EQ(testWidth, result[0]);
    EXPECT_EQ(testHeight, result[1]);
}

// ============================================================================
// WINDOW COMPUTATION TESTS
// ============================================================================

TEST_F(RegionsOfInterestAlgorithmTest, ComputeWindowNoWindowDefined) {
    // When no window is defined (center is zero or dimensions are zero),
    // the window should default to the entire image
    algorithm.setImageSize(1000, 2000);
    EXPECT_NO_THROW(algorithm.reset());
    auto center = algorithm.getWindowCenter();
    auto size = algorithm.getWindowSize();
    EXPECT_EQ(center.x(), 1000 / 2);
    EXPECT_EQ(center.y(), 2000 / 2);
    EXPECT_EQ(size.x(), 1000);
    EXPECT_EQ(size.y(), 2000);
}

TEST_F(RegionsOfInterestAlgorithmTest, ComputeWindowValidWindow) {
    // Set up a valid window centered in the image
    Eigen::Vector2i center(512, 384);
    algorithm.setWindowCenter(center);
    algorithm.setWindowSize(400, 300);
    algorithm.setImageSize(1000, 1000);

    EXPECT_NO_THROW(algorithm.reset());
}

TEST_F(RegionsOfInterestAlgorithmTest, ComputeWindowExtendsLeftThrows) {
    // Window extends beyond left edge
    Eigen::Vector2i center(50, 384);
    algorithm.setWindowCenter(center);
    algorithm.setWindowSize(200, 300);
    algorithm.setImageSize(1000, 1000);

    EXPECT_THROW(algorithm.reset(), std::invalid_argument);
}

TEST_F(RegionsOfInterestAlgorithmTest, ComputeWindowExtendsTopThrows) {
    // Window extends beyond top edge
    Eigen::Vector2i center(512, 50);
    algorithm.setWindowCenter(center);
    algorithm.setWindowSize(400, 200);
    algorithm.setImageSize(1000, 1000);

    EXPECT_THROW(algorithm.reset(), std::invalid_argument);
}

TEST_F(RegionsOfInterestAlgorithmTest, ComputeWindowExtendsRightThrows) {
    // Window extends beyond right edge (assuming 1024 width image)
    Eigen::Vector2i center(900, 384);
    algorithm.setWindowCenter(center);
    algorithm.setWindowSize(400, 300);
    algorithm.setImageSize(1000, 1000);

    EXPECT_THROW(algorithm.reset(), std::invalid_argument);
}

TEST_F(RegionsOfInterestAlgorithmTest, ComputeWindowExtendsBottomThrows) {
    // Window extends beyond bottom edge (assuming 768 height image)
    Eigen::Vector2i center(512, 750);
    algorithm.setWindowCenter(center);
    algorithm.setWindowSize(400, 300);
    algorithm.setImageSize(1000, 800);

    EXPECT_THROW(algorithm.reset(), std::invalid_argument);
}

// ============================================================================
// REGION IDENTIFICATION TESTS
// ============================================================================

TEST_F(RegionsOfInterestAlgorithmTest, EmptyRegions) {
    Eigen::Vector2i center(512, 512);
    algorithm.setWindowCenter(center);
    algorithm.setWindowSize(400, 300);
    algorithm.setImageSize(1024, 1024);
    algorithm.reset();
    // All regions have zero pixels - should return empty region
    std::array<RegionOfInterest, MAX_NUMBER_REGIONS> regions{};

    RegionOfInterest result = algorithm.update(regions);

    EXPECT_EQ(0, result.numberOfPixels);
}

TEST_F(RegionsOfInterestAlgorithmTest, SingleRegion) {
    Eigen::Vector2i center(512, 512);
    algorithm.setWindowCenter(center);
    algorithm.setWindowSize(400, 300);
    algorithm.setImageSize(1024, 1024);
    algorithm.reset();
    // Single region above threshold
    std::array<RegionOfInterest, MAX_NUMBER_REGIONS> regions{};

    regions[0].numberOfPixels = 100;
    regions[0].centerOfBrightness << 512, 384;
    regions[0].regionCenter << 512, 384;
    regions[0].regionSize << 50, 50;

    algorithm.setMaxRoiSeparation(500);
    RegionOfInterest result = algorithm.update(regions);

    EXPECT_EQ(100, result.numberOfPixels);
    EXPECT_EQ(512, result.centerOfBrightness[0]);
    EXPECT_EQ(384, result.centerOfBrightness[1]);
}

TEST_F(RegionsOfInterestAlgorithmTest, MultipleRegionsSelectsLargest) {
    Eigen::Vector2i center(512, 512);
    algorithm.setWindowCenter(center);
    algorithm.setWindowSize(900, 900);
    algorithm.setImageSize(1024, 1024);
    algorithm.reset();
    // Multiple regions, should select the largest one if they're far apart
    std::array<RegionOfInterest, MAX_NUMBER_REGIONS> regions{};

    regions[0].numberOfPixels = 100;
    regions[0].centerOfBrightness << 200, 200;

    regions[1].numberOfPixels = 150;
    regions[1].centerOfBrightness << 800, 600;

    regions[2].numberOfPixels = 50;
    regions[2].centerOfBrightness << 400, 400;

    algorithm.setMaxRoiSeparation(50);  // Small separation to prevent merging

    RegionOfInterest result = algorithm.update(regions);

    // Should return the largest region (150 pixels)
    EXPECT_EQ(150, result.numberOfPixels);
    Eigen::Vector2i predictedCenter = regions[1].centerOfBrightness;
    for (auto i = 0; i < 2; ++i) {
        EXPECT_EQ(predictedCenter[i], result.centerOfBrightness[i]);
    }
}

TEST_F(RegionsOfInterestAlgorithmTest, CloseRegionsMerges) {
    Eigen::Vector2i center(512, 512);
    algorithm.setWindowCenter(center);
    algorithm.setWindowSize(400, 300);
    algorithm.setImageSize(1024, 1024);
    algorithm.reset();
    // Two regions close together should be merged
    std::array<RegionOfInterest, MAX_NUMBER_REGIONS> regions{};

    regions[0].numberOfPixels = 100;
    regions[0].centerOfBrightness << 500, 400;

    regions[1].numberOfPixels = 80;
    regions[1].centerOfBrightness << 510, 405;

    algorithm.setMaxRoiSeparation(50);  // Close enough to merge

    RegionOfInterest result = algorithm.update(regions);

    // Should merge: total pixels = 180
    EXPECT_EQ(180, result.numberOfPixels);
    Eigen::Vector2i predictedCenter = (regions[0].numberOfPixels * regions[0].centerOfBrightness +
                                       regions[1].numberOfPixels * regions[1].centerOfBrightness) /
                                      (regions[0].numberOfPixels + regions[1].numberOfPixels);
    for (auto i = 0; i < 2; ++i) {
        EXPECT_EQ(predictedCenter[i], result.centerOfBrightness[i]);
    }
}

TEST_F(RegionsOfInterestAlgorithmTest, ThreeRegionsTwoCloseMerges) {
    Eigen::Vector2i center(512, 512);
    algorithm.setWindowCenter(center);
    algorithm.setWindowSize(400, 300);
    algorithm.setImageSize(1024, 1024);
    algorithm.reset();
    // Three regions: two close together, one far away
    std::array<RegionOfInterest, MAX_NUMBER_REGIONS> regions{};

    regions[0].numberOfPixels = 100;
    regions[0].centerOfBrightness << 500, 400;

    regions[1].numberOfPixels = 80;
    regions[1].centerOfBrightness << 510, 405;

    regions[2].numberOfPixels = 50;
    regions[2].centerOfBrightness << 800, 600;  // Far away

    algorithm.setMaxRoiSeparation(50);

    RegionOfInterest result = algorithm.update(regions);

    // Should merge the two close regions (180 total pixels)
    EXPECT_EQ(regions[0].numberOfPixels + regions[1].numberOfPixels, result.numberOfPixels);
    Eigen::Vector2i predictedCenter = (regions[0].numberOfPixels * regions[0].centerOfBrightness +
                                       regions[1].numberOfPixels * regions[1].centerOfBrightness) /
                                      (regions[0].numberOfPixels + regions[1].numberOfPixels);
    for (auto i = 0; i < 2; ++i) {
        EXPECT_EQ(predictedCenter[i], result.centerOfBrightness[i]);
    }
}

TEST_F(RegionsOfInterestAlgorithmTest, FiltersRegionsBelowThreshold) {
    Eigen::Vector2i center(512, 512);
    algorithm.setWindowCenter(center);
    algorithm.setWindowSize(400, 300);
    algorithm.setImageSize(1024, 1024);
    algorithm.reset();
    // Regions below minimum detection threshold should be filtered
    std::array<RegionOfInterest, MAX_NUMBER_REGIONS> regions{};

    algorithm.setMinimumDetectionSize(10);

    regions[0].numberOfPixels = 5;  // Below threshold
    regions[0].centerOfBrightness << 500, 400;

    regions[1].numberOfPixels = 2;  // Below threshold
    regions[1].centerOfBrightness << 510, 405;

    RegionOfInterest result = algorithm.update(regions);

    // Should return empty region since all are below threshold
    EXPECT_EQ(0, result.numberOfPixels);
    Eigen::Vector2i predictedCenter = Eigen::Vector2i::Zero();
    for (auto i = 0; i < 2; ++i) {
        EXPECT_EQ(predictedCenter[i], result.centerOfBrightness[i]);
    }
}

TEST_F(RegionsOfInterestAlgorithmTest, ExludeRegionsBelowThreshold) {
    Eigen::Vector2i center(512, 512);
    algorithm.setWindowCenter(center);
    algorithm.setWindowSize(400, 300);
    algorithm.setImageSize(1024, 1024);
    algorithm.reset();
    // Regions below minimum detection threshold should be filtered
    std::array<RegionOfInterest, MAX_NUMBER_REGIONS> regions{};

    algorithm.setMinimumDetectionSize(10);

    regions[0].numberOfPixels = 5;  // Below threshold
    regions[0].centerOfBrightness << 500, 400;

    regions[1].numberOfPixels = 2;  // Below threshold
    regions[1].centerOfBrightness << 510, 405;

    regions[2].numberOfPixels = 20;
    regions[2].centerOfBrightness << 520, 415;

    RegionOfInterest result = algorithm.update(regions);

    // Should return empty region since all are below threshold
    EXPECT_EQ(20, result.numberOfPixels);
    Eigen::Vector2i predictedCenter = regions[2].centerOfBrightness;
    for (auto i = 0; i < 2; ++i) {
        EXPECT_EQ(predictedCenter[i], result.centerOfBrightness[i]);
    }
}

// ============================================================================
// WINDOWING TESTS
// ============================================================================

TEST_F(RegionsOfInterestAlgorithmTest, WindowingFiltersOutsideRegions) {
    // Set up a window
    Eigen::Vector2i windowCenter(512, 384);
    algorithm.setWindowCenter(windowCenter);
    algorithm.setWindowSize(400, 300);
    algorithm.setImageSize(1024, 1024);
    algorithm.reset();

    std::array<RegionOfInterest, MAX_NUMBER_REGIONS> regions{};

    // Region inside window
    regions[0].numberOfPixels = 100;
    regions[0].centerOfBrightness << 500, 380;

    // Region outside window
    regions[1].numberOfPixels = 150;
    regions[1].centerOfBrightness << 100, 100;

    RegionOfInterest result = algorithm.update(regions);

    // Should only consider the region inside the window (100 pixels)
    EXPECT_EQ(100, result.numberOfPixels);
    Eigen::Vector2i predictedCenter = regions[0].centerOfBrightness;
    for (auto i = 0; i < 2; ++i) {
        EXPECT_EQ(predictedCenter[i], result.centerOfBrightness[i]);
    }
}

TEST_F(RegionsOfInterestAlgorithmTest, WindowingAllRegionsOutsideReturnsEmpty) {
    // Set up a window
    Eigen::Vector2i windowCenter(512, 384);
    algorithm.setWindowCenter(windowCenter);
    algorithm.setWindowSize(200, 150);
    algorithm.reset();

    std::array<RegionOfInterest, MAX_NUMBER_REGIONS> regions{};

    // All regions outside window
    regions[0].numberOfPixels = 100;
    regions[0].centerOfBrightness << 100, 100;

    regions[1].numberOfPixels = 150;
    regions[1].centerOfBrightness << 800, 700;

    RegionOfInterest result = algorithm.update(regions);

    // Should return empty region
    EXPECT_EQ(0, result.numberOfPixels);
    Eigen::Vector2i predictedCenter = Eigen::Vector2i::Zero();
    for (auto i = 0; i < 2; ++i) {
        EXPECT_EQ(predictedCenter[i], result.centerOfBrightness[i]);
    }
}

// ============================================================================
// REGION ORDERING TESTS
// ============================================================================

TEST_F(RegionsOfInterestAlgorithmTest, RegionsOrderedByPixelCount) {
    // Verify that regions are processed in descending order of pixel count
    std::array<RegionOfInterest, MAX_NUMBER_REGIONS> regions{};

    regions[0].numberOfPixels = 50;
    regions[0].centerOfBrightness << 200, 200;

    regions[1].numberOfPixels = 150;  // Largest
    regions[1].centerOfBrightness << 500, 400;

    regions[2].numberOfPixels = 100;
    regions[2].centerOfBrightness << 300, 300;

    algorithm.setMaxRoiSeparation(10);  // Small separation to prevent merging

    RegionOfInterest result = algorithm.update(regions);

    // Should select the largest region (150 pixels) even though it's not first
    EXPECT_EQ(150, result.numberOfPixels);
    Eigen::Vector2i predictedCenter = regions[1].centerOfBrightness;
    for (auto i = 0; i < 2; ++i) {
        EXPECT_EQ(predictedCenter[i], result.centerOfBrightness[i]);
    }
}

// ============================================================================
// EDGE CASE TESTS
// ============================================================================

TEST_F(RegionsOfInterestAlgorithmTest, MaxNumberOfRegions) {
    // Fill all possible regions
    std::array<RegionOfInterest, MAX_NUMBER_REGIONS> regions{};

    for (size_t i = 0; i < MAX_NUMBER_REGIONS; ++i) {
        regions[i].numberOfPixels = 10 + i;
        regions[i].centerOfBrightness << 100 + i * 10, 100 + i * 10;
    }

    EXPECT_NO_THROW(algorithm.update(regions));
}

TEST_F(RegionsOfInterestAlgorithmTest, ResetClearsState) {
    // Verify that reset properly reinitializes the algorithm
    algorithm.setWindowCenter(Eigen::Vector2i(512, 384));
    algorithm.setWindowSize(400, 300);

    EXPECT_NO_THROW(algorithm.reset());

    // After reset, algorithm should still work
    std::array<RegionOfInterest, MAX_NUMBER_REGIONS> regions{};
    regions[0].numberOfPixels = 100;
    regions[0].centerOfBrightness << 500, 380;

    EXPECT_NO_THROW(algorithm.update(regions));
}

TEST_F(RegionsOfInterestAlgorithmTest, RegionSizeBasedOnMaxSeparation) {
    // Verify that the output region size is correctly computed
    std::array<RegionOfInterest, MAX_NUMBER_REGIONS> regions{};

    regions[0].numberOfPixels = 100;
    regions[0].centerOfBrightness << 500, 400;

    int32_t maxSep = 100;
    algorithm.setMaxRoiSeparation(maxSep);

    RegionOfInterest result = algorithm.update(regions);

    // Region size should be maxSeparation / sqrt(2) for both width and height
    int32_t expectedSize = std::floor(maxSep / std::sqrt(2));
    EXPECT_EQ(expectedSize, result.regionSize[0]);
    EXPECT_EQ(expectedSize, result.regionSize[1]);
}

// ============================================================================
// INTEGRATION TESTS WITH REALISTIC SCENARIOS
// ============================================================================

TEST(RegionsOfInterestTest, SingleBrightTarget) {
    // Simulate a single bright target detection
    std::array<RegionOfInterest, MAX_NUMBER_REGIONS> regions{};

    regions[0].numberOfPixels = 250;
    regions[0].centerOfBrightness << 512, 384;
    regions[0].regionCenter << 512, 384;
    regions[0].regionSize << 20, 20;
    regions[0].timeTag = 1.0;

    EXPECT_NO_THROW(testRegionIdentification(regions, Eigen::Vector2i(512, 384), 250));
}

TEST(RegionsOfInterestTest, MultipleTargetsSelectBrightest) {
    // Multiple targets, should select the brightest
    std::array<RegionOfInterest, MAX_NUMBER_REGIONS> regions{};

    regions[0].numberOfPixels = 150;
    regions[0].centerOfBrightness << 300, 300;

    regions[1].numberOfPixels = 250;  // Brightest
    regions[1].centerOfBrightness << 512, 384;

    regions[2].numberOfPixels = 100;
    regions[2].centerOfBrightness << 700, 500;

    EXPECT_NO_THROW(testRegionIdentification(regions,
                                             Eigen::Vector2i(512, 384),
                                             250,
                                             Eigen::Vector2i::Zero(),
                                             0,
                                             0,
                                             50));  // Small separation
}

TEST(RegionsOfInterestTest, SplitDetectionMerges) {
    // Object split into two detections due to saturation
    std::array<RegionOfInterest, MAX_NUMBER_REGIONS> regions{};

    regions[0].numberOfPixels = 120;
    regions[0].centerOfBrightness << 510, 382;

    regions[1].numberOfPixels = 130;
    regions[1].centerOfBrightness << 514, 386;

    // Close separation should merge them
    RegionsOfInterestAlgorithm algorithm;
    algorithm.setMaxRoiSeparation(50);

    RegionOfInterest result = algorithm.update(regions);

    // Should merge to 250 total pixels
    EXPECT_EQ(250, result.numberOfPixels);
    Eigen::Vector2i predictedCenter = (regions[0].numberOfPixels * regions[0].centerOfBrightness +
                                       regions[1].numberOfPixels * regions[1].centerOfBrightness) /
                                      (regions[0].numberOfPixels + regions[1].numberOfPixels);
    for (auto i = 0; i < 2; ++i) {
        EXPECT_EQ(predictedCenter[i], result.centerOfBrightness[i]);
    }
}

TEST(RegionsOfInterestTest, WindowedDetectionFiltersSurroundings) {
    // Use windowing to focus on a specific region of the sky
    std::array<RegionOfInterest, MAX_NUMBER_REGIONS> regions{};

    // Target star in window
    regions[0].numberOfPixels = 150;
    regions[0].centerOfBrightness << 512, 384;

    // Bright star outside window
    regions[1].numberOfPixels = 300;
    regions[1].centerOfBrightness << 100, 100;

    EXPECT_NO_THROW(
        testRegionIdentification(regions, Eigen::Vector2i(512, 384), 150, Eigen::Vector2i(512, 384), 400, 300));
}
