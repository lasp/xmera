// SPDX-License-Identifier: ISC
// Copyright (c) 2026, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

#include "../regionsOfInterestAlgorithm.h"
#include "gtest/gtest.h"
#include "test_regionsOfInterest.cpp"
#include <fuzztest/fuzztest.h>

#include <cmath>

// Fuzz-specific tolerances
constexpr int32_t FUZZ_MAX_IMAGE_SIZE = 4096;
constexpr int32_t FUZZ_MIN_IMAGE_SIZE = 64;
constexpr int32_t FUZZ_MAX_PIXELS = 10000;

/*! @brief Main fuzz test for RegionsOfInterestAlgorithm
 *
 *  Tests the algorithm across a wide range of region configurations.
 *  Ensures the algorithm doesn't crash and produces reasonable results.
 */
void fuzzRegionIdentification(int32_t numRegions,
                              int32_t maxSeparation,
                              int32_t minDetectionSize,
                              int32_t region1_x,
                              int32_t region1_y,
                              int32_t region1_pixels,
                              int32_t region2_x,
                              int32_t region2_y,
                              int32_t region2_pixels) {
    std::array<RegionOfInterest, MAX_NUMBER_REGIONS> regions{};

    // Set up first region if numRegions >= 1
    if (numRegions >= 1) {
        regions[0].numberOfPixels = region1_pixels;
        regions[0].centerOfBrightness << region1_x, region1_y;
        regions[0].regionCenter << region1_x, region1_y;
        regions[0].regionSize << 10, 10;
    }

    // Set up second region if numRegions >= 2
    if (numRegions >= 2) {
        regions[1].numberOfPixels = region2_pixels;
        regions[1].centerOfBrightness << region2_x, region2_y;
        regions[1].regionCenter << region2_x, region2_y;
        regions[1].regionSize << 10, 10;
    }

    RegionsOfInterestAlgorithm algorithm;
    algorithm.setMaxRoiSeparation(maxSeparation);
    algorithm.setMinimumDetectionSize(minDetectionSize);

    // Algorithm should not crash regardless of input
    RegionOfInterest result = algorithm.update(regions);

    // Result should have non-negative pixel count
    EXPECT_GE(result.numberOfPixels, 0);

    // If result has pixels, it should have valid region size
    if (result.numberOfPixels > 0) {
        EXPECT_GE(result.regionSize[0], 0);
        EXPECT_GE(result.regionSize[1], 0);
    };
}

FUZZ_TEST(RegionsOfInterestFuzz, fuzzRegionIdentification)
    .WithDomains(fuzztest::InRange(0, 5),                    // numRegions
                 fuzztest::InRange(1, 2000),                 // maxSeparation
                 fuzztest::InRange(0, 100),                  // minDetectionSize
                 fuzztest::InRange(0, FUZZ_MAX_IMAGE_SIZE),  // region1_x
                 fuzztest::InRange(0, FUZZ_MAX_IMAGE_SIZE),  // region1_y
                 fuzztest::InRange(0, FUZZ_MAX_PIXELS),      // region1_pixels
                 fuzztest::InRange(0, FUZZ_MAX_IMAGE_SIZE),  // region2_x
                 fuzztest::InRange(0, FUZZ_MAX_IMAGE_SIZE),  // region2_y
                 fuzztest::InRange(0, FUZZ_MAX_PIXELS));     // region2_pixels

/*! @brief Fuzz test for windowing functionality
 *
 *  Tests the windowing feature with various window configurations.
 */
void fuzzWindowingBehavior(int32_t window_center_x,
                           int32_t window_center_y,
                           int32_t window_width,
                           int32_t window_height,
                           int32_t region_x,
                           int32_t region_y,
                           int32_t region_pixels) {
    RegionsOfInterestAlgorithm algorithm;
    algorithm.setImageSize(FUZZ_MAX_IMAGE_SIZE, FUZZ_MAX_IMAGE_SIZE);
    // Set window parameters
    if (window_width > 0 && window_height > 0) {
        Eigen::Vector2i windowCenter(window_center_x, window_center_y);
        algorithm.setWindowCenter(windowCenter);
        algorithm.setWindowSize(window_width, window_height);

        EXPECT_NO_THROW(algorithm.reset());
    } else {
        EXPECT_NO_THROW(algorithm.reset());
        Eigen::Vector2i windowCenter = algorithm.getWindowCenter();
        Eigen::Vector2i windowSize = algorithm.getWindowSize();
        EXPECT_EQ(windowCenter.x(), FUZZ_MAX_IMAGE_SIZE / 2);
        EXPECT_EQ(windowCenter.y(), FUZZ_MAX_IMAGE_SIZE / 2);
        EXPECT_EQ(windowSize.x(), FUZZ_MAX_IMAGE_SIZE);
        EXPECT_EQ(windowSize.y(), FUZZ_MAX_IMAGE_SIZE);
    }

    std::array<RegionOfInterest, MAX_NUMBER_REGIONS> regions{};
    regions[0].numberOfPixels = region_pixels;
    regions[0].centerOfBrightness << region_x, region_y;
    regions[0].regionCenter << region_x, region_y;

    RegionOfInterest result = algorithm.update(regions);
    EXPECT_GE(result.numberOfPixels, 0);
}

FUZZ_TEST(RegionsOfInterestFuzz, fuzzWindowingBehavior)
    .WithDomains(fuzztest::InRange(401, FUZZ_MAX_IMAGE_SIZE - 401),  // window_center_x
                 fuzztest::InRange(301, FUZZ_MAX_IMAGE_SIZE - 301),  // window_center_y
                 fuzztest::InRange(0, 800),                          // window_width
                 fuzztest::InRange(0, 600),                          // window_height
                 fuzztest::InRange(0, FUZZ_MAX_IMAGE_SIZE),          // region_x
                 fuzztest::InRange(0, FUZZ_MAX_IMAGE_SIZE),          // region_y
                 fuzztest::InRange(0, FUZZ_MAX_PIXELS));             // region_pixels

/*! @brief Fuzz test for region merging logic
 *
 *  Tests scenarios where multiple regions might be merged based on proximity.
 */
void fuzzRegionMerging(int32_t maxSeparation,
                       int32_t region1_x,
                       int32_t region1_y,
                       int32_t region1_pixels,
                       int32_t region2_x,
                       int32_t region2_y,
                       int32_t region2_pixels,
                       int32_t region3_x,
                       int32_t region3_y,
                       int32_t region3_pixels) {
    std::array<RegionOfInterest, MAX_NUMBER_REGIONS> regions{};

    regions[0].numberOfPixels = region1_pixels;
    regions[0].centerOfBrightness << region1_x, region1_y;
    regions[0].regionCenter << region1_x, region1_y;
    regions[0].regionSize << 10, 10;

    regions[1].numberOfPixels = region2_pixels;
    regions[1].centerOfBrightness << region2_x, region2_y;
    regions[1].regionCenter << region2_x, region2_y;
    regions[1].regionSize << 10, 10;

    regions[2].numberOfPixels = region3_pixels;
    regions[2].centerOfBrightness << region3_x, region3_y;
    regions[2].regionCenter << region3_x, region3_y;
    regions[2].regionSize << 10, 10;

    RegionsOfInterestAlgorithm algorithm;
    algorithm.setMaxRoiSeparation(maxSeparation);

    RegionOfInterest result = algorithm.update(regions);

    // Verify result invariants
    EXPECT_GE(result.numberOfPixels, 0);

    // If regions were merged, total pixels should not exceed sum of all inputs
    int32_t totalInputPixels = region1_pixels + region2_pixels + region3_pixels;
    EXPECT_LE(result.numberOfPixels, totalInputPixels);

    // Region size should be consistent with maxSeparation
    if (result.numberOfPixels > 0) {
        int32_t expectedSize = std::floor(maxSeparation / std::sqrt(2));
        EXPECT_EQ(expectedSize, result.regionSize[0]);
        EXPECT_EQ(expectedSize, result.regionSize[1]);
    };
}

FUZZ_TEST(RegionsOfInterestFuzz, fuzzRegionMerging)
    .WithDomains(fuzztest::InRange(10, 1000),  // maxSeparation
                 fuzztest::InRange(0, 1024),   // region1_x
                 fuzztest::InRange(0, 768),    // region1_y
                 fuzztest::InRange(0, 1000),   // region1_pixels
                 fuzztest::InRange(0, 1024),   // region2_x
                 fuzztest::InRange(0, 768),    // region2_y
                 fuzztest::InRange(0, 1000),   // region2_pixels
                 fuzztest::InRange(0, 1024),   // region3_x
                 fuzztest::InRange(0, 768),    // region3_y
                 fuzztest::InRange(0, 1000));  // region3_pixels

/*! @brief Fuzz test for edge cases with extreme values
 *
 *  Tests the algorithm's robustness with boundary conditions.
 */
void fuzzEdgeCases(int32_t maxSeparation, int32_t minDetectionSize) {
    RegionsOfInterestAlgorithm algorithm;

    // Test with extreme parameter values
    algorithm.setMaxRoiSeparation(maxSeparation);
    algorithm.setMinimumDetectionSize(minDetectionSize);

    // Create regions at extreme positions
    std::array<RegionOfInterest, MAX_NUMBER_REGIONS> regions{};

    // Region at origin
    regions[0].numberOfPixels = minDetectionSize + 10;
    regions[0].centerOfBrightness << 0, 0;

    // Region at max coordinates
    regions[1].numberOfPixels = minDetectionSize + 20;
    regions[1].centerOfBrightness << FUZZ_MAX_IMAGE_SIZE, FUZZ_MAX_IMAGE_SIZE;

    RegionOfInterest result = algorithm.update(regions);
    EXPECT_GE(result.numberOfPixels, 0);
}

FUZZ_TEST(RegionsOfInterestFuzz, fuzzEdgeCases)
    .WithDomains(fuzztest::OneOf(fuzztest::InRange(1, 10),      // Very small separation
                                 fuzztest::InRange(100, 500),   // Normal separation
                                 fuzztest::InRange(1000, 5000)  // Very large separation
                                 ),
                 fuzztest::OneOf(fuzztest::InRange(0, 5),    // Very small threshold
                                 fuzztest::InRange(5, 50),   // Normal threshold
                                 fuzztest::InRange(50, 500)  // Very large threshold
                                 ));

/*! @brief Fuzz test for full region array
 *
 *  Tests with the maximum number of regions to ensure no overflow issues.
 */
void fuzzFullRegionArray(int32_t maxSeparation, int32_t basePixelCount, int32_t positionSpread) {
    std::array<RegionOfInterest, MAX_NUMBER_REGIONS> regions{};

    // Fill all region slots
    for (size_t i = 0; i < MAX_NUMBER_REGIONS; ++i) {
        regions[i].numberOfPixels = basePixelCount + static_cast<int32_t>(i);
        regions[i].centerOfBrightness << (i * 10) % positionSpread, (i * 15) % positionSpread;
        regions[i].regionCenter = regions[i].centerOfBrightness;
        regions[i].regionSize << 10, 10;
    }

    RegionsOfInterestAlgorithm algorithm;
    algorithm.setMaxRoiSeparation(maxSeparation);

    RegionOfInterest result = algorithm.update(regions);
    EXPECT_GE(result.numberOfPixels, 0);
}

FUZZ_TEST(RegionsOfInterestFuzz, fuzzFullRegionArray)
    .WithDomains(fuzztest::InRange(10, 500),     // maxSeparation
                 fuzztest::InRange(1, 100),      // basePixelCount
                 fuzztest::InRange(100, 2048));  // positionSpread
