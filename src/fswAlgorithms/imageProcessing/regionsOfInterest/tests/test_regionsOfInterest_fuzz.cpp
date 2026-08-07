// SPDX-License-Identifier: ISC
// Copyright (c) 2026, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

#include "../regionsOfInterestAlgorithm.h"
#include "gtest/gtest.h"
#include <fuzztest/fuzztest.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <stdexcept>
#include <vector>

// Fuzz-specific tolerances
constexpr int32_t FUZZ_MAX_IMAGE_SIZE = 4096;
constexpr int32_t FUZZ_MIN_IMAGE_SIZE = 64;
constexpr int32_t FUZZ_MAX_PIXELS = 10000;

/*! @brief Number of regions the fuzzed vectors can describe */
constexpr size_t FUZZ_MAX_REGIONS = static_cast<size_t>(MAX_NUMBER_REGIONS);

/*! @brief Bounds of the region centers that are more than the detection threshold
 *
 *  A region at or below the threshold supplies nothing to the identified ROI. Thus it must
 *  not increase the size of the bounds that the test compares the result against.
 */
struct CenterBounds {
    bool valid{false};
    int32_t minX{};
    int32_t maxX{};
    int32_t minY{};
    int32_t maxY{};
};

inline CenterBounds
boundsOfDetectedRegions(std::array<RegionOfInterest, MAX_NUMBER_REGIONS> const &regions, int32_t minDetectionSize) {
    CenterBounds bounds{};
    for (auto const &region : regions) {
        if (region.numberOfPixels <= minDetectionSize) { continue; }
        int32_t const x = region.centerOfBrightness[0];
        int32_t const y = region.centerOfBrightness[1];
        if (!bounds.valid) {
            bounds = CenterBounds{true, x, x, y, y};
            continue;
        }
        bounds.minX = std::min(bounds.minX, x);
        bounds.maxX = std::max(bounds.maxX, x);
        bounds.minY = std::min(bounds.minY, y);
        bounds.maxY = std::max(bounds.maxY, y);
    }
    return bounds;
}

/*! @brief Main fuzz test for RegionsOfInterestAlgorithm
 *
 *  Tests the algorithm across a wide range of region configurations.
 *  Ensures the algorithm doesn't crash and produces reasonable results.
 */
void fuzzRegionIdentification(
    int32_t maxSeparation,
    int32_t minDetectionSize,
    std::vector<int32_t> regionXs,
    std::vector<int32_t> regionYs,
    std::vector<int32_t> regionPixels
) {
    size_t const numRegions = std::min({regionXs.size(), regionYs.size(), regionPixels.size(), FUZZ_MAX_REGIONS});

    std::array<RegionOfInterest, MAX_NUMBER_REGIONS> regions{};
    int32_t totalInputPixels = 0;
    for (size_t i = 0; i < numRegions; ++i) {
        regions[i].numberOfPixels = regionPixels[i];
        regions[i].centerOfBrightness << regionXs[i], regionYs[i];
        regions[i].regionCenter << regionXs[i], regionYs[i];
        totalInputPixels += regionPixels[i];
    }

    RegionsOfInterestAlgorithm algorithm;
    algorithm.setImageSize(FUZZ_MAX_IMAGE_SIZE, FUZZ_MAX_IMAGE_SIZE);
    algorithm.setMaxRoiSeparation(maxSeparation);
    algorithm.setMinimumDetectionSize(minDetectionSize);
    // Without reset() the window keeps its default size of 1024x1024 and rejects each
    // region outside it, whatever image size the test sets above.
    algorithm.reset();

    // Algorithm should not crash regardless of input
    RegionOfInterest result = algorithm.update(regions);

    // The result must have a pixel count that is not negative, and it cannot add pixels
    EXPECT_GE(result.numberOfPixels, 0);
    EXPECT_LE(result.numberOfPixels, totalInputPixels);

    // If result has pixels, it should have valid region size
    if (result.numberOfPixels > 0) {
        EXPECT_GE(result.regionSize[0], 0);
        EXPECT_GE(result.regionSize[1], 0);
    }
}

FUZZ_TEST(RegionsOfInterestFuzz, fuzzRegionIdentification)
    .WithDomains(
        fuzztest::InRange(1, 2'000),  // maxSeparation
        fuzztest::InRange(0, 100),    // minDetectionSize
        // The length of the fuzzed vector gives the region count. Thus each slot
        // of the MAX_NUMBER_REGIONS array is available, and zero slots also.
        fuzztest::VectorOf(fuzztest::InRange(0, FUZZ_MAX_IMAGE_SIZE)).WithMaxSize(FUZZ_MAX_REGIONS),
        fuzztest::VectorOf(fuzztest::InRange(0, FUZZ_MAX_IMAGE_SIZE)).WithMaxSize(FUZZ_MAX_REGIONS),
        fuzztest::VectorOf(fuzztest::InRange(0, FUZZ_MAX_PIXELS)).WithMaxSize(FUZZ_MAX_REGIONS)
    );

/*! @brief Fuzz test for windowing functionality
 *
 *  Tests the window function with many window configurations. These include windows that
 *  extend past each image edge, which the algorithm must reject.
 */
void fuzzWindowingBehavior(
    int32_t windowCenterX,
    int32_t windowCenterY,
    int32_t windowWidth,
    int32_t windowHeight,
    int32_t regionX,
    int32_t regionY,
    int32_t regionPixels
) {
    RegionsOfInterestAlgorithm algorithm;
    algorithm.setImageSize(FUZZ_MAX_IMAGE_SIZE, FUZZ_MAX_IMAGE_SIZE);

    Eigen::Vector2i const windowCenter(windowCenterX, windowCenterY);
    algorithm.setWindowCenter(windowCenter);
    algorithm.setWindowSize(windowWidth, windowHeight);

    // If the center is zero or a dimension is zero, computeWindow uses the full image,
    // which always fits. Each other window must stay inside the image, or reset() throws.
    bool const wholeImage = windowCenter.isZero() || windowWidth == 0 || windowHeight == 0;
    bool const fits = wholeImage
                   || (windowCenterX - windowWidth / 2 >= 0 && windowCenterY - windowHeight / 2 >= 0
                       && windowCenterX + windowWidth / 2 <= FUZZ_MAX_IMAGE_SIZE
                       && windowCenterY + windowHeight / 2 <= FUZZ_MAX_IMAGE_SIZE);

    if (!fits) {
        ASSERT_THROW(algorithm.reset(), std::invalid_argument);
        return;  // the window was rejected, so there is no configured algorithm to drive
    }
    ASSERT_NO_THROW(algorithm.reset());

    if (wholeImage) {
        EXPECT_EQ(algorithm.getWindowCenter().x(), FUZZ_MAX_IMAGE_SIZE / 2);
        EXPECT_EQ(algorithm.getWindowCenter().y(), FUZZ_MAX_IMAGE_SIZE / 2);
        EXPECT_EQ(algorithm.getWindowSize().x(), FUZZ_MAX_IMAGE_SIZE);
        EXPECT_EQ(algorithm.getWindowSize().y(), FUZZ_MAX_IMAGE_SIZE);
    } else {
        EXPECT_EQ(algorithm.getWindowCenter().x(), windowCenterX);
        EXPECT_EQ(algorithm.getWindowCenter().y(), windowCenterY);
        EXPECT_EQ(algorithm.getWindowSize().x(), windowWidth);
        EXPECT_EQ(algorithm.getWindowSize().y(), windowHeight);
    }

    std::array<RegionOfInterest, MAX_NUMBER_REGIONS> regions{};
    regions[0].numberOfPixels = regionPixels;
    regions[0].centerOfBrightness << regionX, regionY;
    regions[0].regionCenter << regionX, regionY;

    RegionOfInterest result = algorithm.update(regions);
    EXPECT_GE(result.numberOfPixels, 0);
    EXPECT_LE(result.numberOfPixels, regionPixels);
}

FUZZ_TEST(RegionsOfInterestFuzz, fuzzWindowingBehavior)
    // The window domains use the full range. Thus the fuzzer makes windows that extend
    // past each of the four image edges, and reset() rejects them.
    .WithDomains(
        fuzztest::InRange(0, FUZZ_MAX_IMAGE_SIZE),  // windowCenterX
        fuzztest::InRange(0, FUZZ_MAX_IMAGE_SIZE),  // windowCenterY
        fuzztest::InRange(0, FUZZ_MAX_IMAGE_SIZE),  // windowWidth
        fuzztest::InRange(0, FUZZ_MAX_IMAGE_SIZE),  // windowHeight
        fuzztest::InRange(0, FUZZ_MAX_IMAGE_SIZE),  // regionX
        fuzztest::InRange(0, FUZZ_MAX_IMAGE_SIZE),  // regionY
        fuzztest::InRange(0, FUZZ_MAX_PIXELS)
    );  // regionPixels

/*! @brief Fuzz test for region merging logic
 *
 *  Tests conditions where the algorithm can merge adjacent regions. The coordinates stay
 *  inside the image, thus the window keeps each region. The test can then compare the
 *  merged center directly against the inputs.
 */
void fuzzRegionMerging(int32_t maxSeparation,
                       int32_t minDetectionSize,
                       int32_t region1X,
                       int32_t region1Y,
                       int32_t region1Pixels,
                       int32_t region2X,
                       int32_t region2Y,
                       int32_t region2Pixels,
                       int32_t region3X,
                       int32_t region3Y,
                       int32_t region3Pixels) {
    std::array<RegionOfInterest, MAX_NUMBER_REGIONS> regions{};

    regions[0].numberOfPixels = region1Pixels;
    regions[0].centerOfBrightness << region1X, region1Y;
    regions[0].regionCenter << region1X, region1Y;

    regions[1].numberOfPixels = region2Pixels;
    regions[1].centerOfBrightness << region2X, region2Y;
    regions[1].regionCenter << region2X, region2Y;

    regions[2].numberOfPixels = region3Pixels;
    regions[2].centerOfBrightness << region3X, region3Y;
    regions[2].regionCenter << region3X, region3Y;

    RegionsOfInterestAlgorithm algorithm;
    algorithm.setImageSize(FUZZ_MAX_IMAGE_SIZE, FUZZ_MAX_IMAGE_SIZE);
    algorithm.setMaxRoiSeparation(maxSeparation);
    algorithm.setMinimumDetectionSize(minDetectionSize);
    algorithm.reset();

    RegionOfInterest result = algorithm.update(regions);

    // Verify result invariants
    EXPECT_GE(result.numberOfPixels, 0);

    // If regions were merged, total pixels should not exceed sum of all inputs
    int32_t const totalInputPixels = region1Pixels + region2Pixels + region3Pixels;
    EXPECT_LE(result.numberOfPixels, totalInputPixels);

    // The algorithm gives either the largest region or a merged barycenter. In the two
    // conditions, the result center must stay inside the bounds of the regions that
    // supplied it.
    if (result.numberOfPixels > 0) {
        CenterBounds const bounds = boundsOfDetectedRegions(regions, minDetectionSize);
        ASSERT_TRUE(bounds.valid);
        EXPECT_GE(result.centerOfBrightness[0], bounds.minX);
        EXPECT_LE(result.centerOfBrightness[0], bounds.maxX);
        EXPECT_GE(result.centerOfBrightness[1], bounds.minY);
        EXPECT_LE(result.centerOfBrightness[1], bounds.maxY);
    }
}

FUZZ_TEST(RegionsOfInterestFuzz, fuzzRegionMerging)
    .WithDomains(
        fuzztest::InRange(10, 1'000),                   // maxSeparation
        fuzztest::InRange(0, 100),                      // minDetectionSize
        fuzztest::InRange(1, FUZZ_MAX_IMAGE_SIZE - 1),  // region1X
        fuzztest::InRange(1, FUZZ_MAX_IMAGE_SIZE - 1),  // region1Y
        fuzztest::InRange(0, 1'000),                    // region1Pixels
        fuzztest::InRange(1, FUZZ_MAX_IMAGE_SIZE - 1),  // region2X
        fuzztest::InRange(1, FUZZ_MAX_IMAGE_SIZE - 1),  // region2Y
        fuzztest::InRange(0, 1'000),                    // region2Pixels
        fuzztest::InRange(1, FUZZ_MAX_IMAGE_SIZE - 1),  // region3X
        fuzztest::InRange(1, FUZZ_MAX_IMAGE_SIZE - 1),  // region3Y
        fuzztest::InRange(0, 1'000)
    );  // region3Pixels

/*! @brief Fuzz test for edge cases with extreme values
 *
 *  Tests the algorithm with boundary conditions. These are extreme separation values and
 *  threshold values, with regions at the image corners and at the image center.
 */
void fuzzEdgeCases(
    int32_t maxSeparation,
    int32_t minDetectionSize,
    int32_t regionAX,
    int32_t regionAY,
    int32_t regionAPixels,
    int32_t regionBX,
    int32_t regionBY,
    int32_t regionBPixels
) {
    RegionsOfInterestAlgorithm algorithm;

    // Test with extreme parameter values
    algorithm.setImageSize(FUZZ_MAX_IMAGE_SIZE, FUZZ_MAX_IMAGE_SIZE);
    algorithm.setMaxRoiSeparation(maxSeparation);
    algorithm.setMinimumDetectionSize(minDetectionSize);
    algorithm.reset();

    std::array<RegionOfInterest, MAX_NUMBER_REGIONS> regions{};

    regions[0].numberOfPixels = regionAPixels;
    regions[0].centerOfBrightness << regionAX, regionAY;

    regions[1].numberOfPixels = regionBPixels;
    regions[1].centerOfBrightness << regionBX, regionBY;

    RegionOfInterest result = algorithm.update(regions);
    EXPECT_GE(result.numberOfPixels, 0);
    EXPECT_LE(result.numberOfPixels, regionAPixels + regionBPixels);
}

// The position domains give values at the image edges, adjacent to the edges, and at the
// middle. regionInWindow uses strict inequalities, thus a value at the edge is outside.
static auto edgeCasePosition() {
    return fuzztest::OneOf(
        fuzztest::InRange(0, 1),
        fuzztest::InRange(FUZZ_MIN_IMAGE_SIZE, FUZZ_MAX_IMAGE_SIZE / 2),
        fuzztest::InRange(FUZZ_MAX_IMAGE_SIZE - 1, FUZZ_MAX_IMAGE_SIZE)
    );
}

FUZZ_TEST(RegionsOfInterestFuzz, fuzzEdgeCases)
    .WithDomains(
        fuzztest::OneOf(
            fuzztest::InRange(1, 10),        // Very small separation
            fuzztest::InRange(100, 500),     // Normal separation
            fuzztest::InRange(1'000, 5'000)  // Very large separation
        ),
        fuzztest::OneOf(
            fuzztest::InRange(0, 4),    // Very small threshold
            fuzztest::InRange(5, 49),   // Normal threshold
            fuzztest::InRange(50, 500)  // Very large threshold
        ),
        edgeCasePosition(),                     // regionAX
        edgeCasePosition(),                     // regionAY
        fuzztest::InRange(0, FUZZ_MAX_PIXELS),  // regionAPixels
        edgeCasePosition(),                     // regionBX
        edgeCasePosition(),                     // regionBY
        fuzztest::InRange(0, FUZZ_MAX_PIXELS)
    );  // regionBPixels

/*! @brief Fuzz test for full region array
 *
 *  Tests with the maximum number of regions to ensure no overflow issues.
 */
void fuzzFullRegionArray(
    int32_t maxSeparation,
    int32_t basePixelCount,
    int32_t positionStride,
    int32_t originX,
    int32_t originY
) {
    std::array<RegionOfInterest, MAX_NUMBER_REGIONS> regions{};

    // Fill all region slots. The stride sets the distance between the regions, thus it
    // multiplies the offset. As a modulus, the small indices can never reach it.
    int32_t totalInputPixels = 0;
    for (size_t i = 0; i < FUZZ_MAX_REGIONS; ++i) {
        auto const step = static_cast<int32_t>(i);
        regions[i].numberOfPixels = basePixelCount + step;
        regions[i].centerOfBrightness << originX + step * positionStride, originY + step * positionStride;
        regions[i].regionCenter = regions[i].centerOfBrightness;
        totalInputPixels += regions[i].numberOfPixels;
    }

    RegionsOfInterestAlgorithm algorithm;
    algorithm.setImageSize(FUZZ_MAX_IMAGE_SIZE, FUZZ_MAX_IMAGE_SIZE);
    algorithm.setMaxRoiSeparation(maxSeparation);
    algorithm.reset();

    RegionOfInterest result = algorithm.update(regions);
    EXPECT_GE(result.numberOfPixels, 0);
    EXPECT_LE(result.numberOfPixels, totalInputPixels);
}

FUZZ_TEST(RegionsOfInterestFuzz, fuzzFullRegionArray)
    // originX/Y plus two strides stays inside the image: 2047 + 2 * 1000 is less than
    // FUZZ_MAX_IMAGE_SIZE.
    .WithDomains(
        fuzztest::InRange(10, 500),                         // maxSeparation
        fuzztest::InRange(1, 100),                          // basePixelCount
        fuzztest::InRange(0, 1'000),                        // positionStride
        fuzztest::InRange(1, FUZZ_MAX_IMAGE_SIZE / 2 - 1),  // originX
        fuzztest::InRange(1, FUZZ_MAX_IMAGE_SIZE / 2 - 1)
    );  // originY
