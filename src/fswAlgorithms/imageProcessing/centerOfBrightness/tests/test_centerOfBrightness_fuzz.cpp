// SPDX-License-Identifier: ISC
// Copyright (c) 2026, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

#include "centerOfBrightnessTestHelpers.hpp"
#include <fuzztest/fuzztest.h>

constexpr int32_t kMaxFuzzPixels = 100;
constexpr int32_t kMaxCoord = 4096;

FUZZ_TEST(CenterOfBrightnessFuzz, fuzzCenterOfBrightness)
    .WithDomains(fuzztest::InRange(0, kMaxCoord),                                               // roiCenterX
                 fuzztest::InRange(0, kMaxCoord),                                               // roiCenterY
                 fuzztest::InRange(1, 1024),                                                    // roiSizeW
                 fuzztest::InRange(1, 1024),                                                    // roiSizeH
                 fuzztest::InRange(0, kMaxFuzzPixels),                                          // numPixels
                 fuzztest::VectorOf(fuzztest::InRange(1, kMaxCoord)).WithSize(kMaxFuzzPixels),  // pixelXs
                 fuzztest::VectorOf(fuzztest::InRange(1, kMaxCoord)).WithSize(kMaxFuzzPixels),  // pixelYs
                 fuzztest::InRange(-1.0, 10.0),                                                 // brightnessThreshold
                 fuzztest::InRange(1, 20)                                                       // avgWindowSize
    );

constexpr int32_t kMaxFuzzSteps = 10;
constexpr int32_t kMaxStepPixels = 50;

FUZZ_TEST(CenterOfBrightnessFuzz, fuzzMultiStepBrightness)
    .WithDomains(fuzztest::InRange(1, 20),       // avgWindowSize
                 fuzztest::InRange(-1.0, 10.0),  // brightnessThreshold
                 fuzztest::VectorOf(fuzztest::InRange(0, kMaxStepPixels)).WithSize(kMaxFuzzSteps)  // pixelCountsPerStep
    );
