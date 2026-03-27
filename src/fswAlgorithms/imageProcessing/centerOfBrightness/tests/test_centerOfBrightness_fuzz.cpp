// SPDX-License-Identifier: ISC
// Copyright (c) 2026, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

#include "centerOfBrightnessTestHelpers.hpp"
#include <fuzztest/fuzztest.h>

constexpr int32_t MAX_FUZZ_PIXELS = 100;
constexpr int32_t MAX_COORD = 4096;

FUZZ_TEST(CenterOfBrightnessFuzz, fuzzCenterOfBrightness)
    .WithDomains(fuzztest::InRange(0, MAX_COORD),                                                // roiCenterX
                 fuzztest::InRange(0, MAX_COORD),                                                // roiCenterY
                 fuzztest::InRange(1, 1024),                                                     // roiSizeW
                 fuzztest::InRange(1, 1024),                                                     // roiSizeH
                 fuzztest::InRange(0, MAX_FUZZ_PIXELS),                                          // numPixels
                 fuzztest::VectorOf(fuzztest::InRange(1, MAX_COORD)).WithSize(MAX_FUZZ_PIXELS),  // pixelXs
                 fuzztest::VectorOf(fuzztest::InRange(1, MAX_COORD)).WithSize(MAX_FUZZ_PIXELS),  // pixelYs
                 fuzztest::InRange(-1.0, 10.0),                                                  // brightnessThreshold
                 fuzztest::InRange(1, 20)                                                        // avgWindowSize
    );

constexpr int32_t MAX_FUZZ_STEPS = 10;
constexpr int32_t MAX_STEP_PIXELS = 50;

FUZZ_TEST(CenterOfBrightnessFuzz, fuzzMultiStepBrightness)
    .WithDomains(
        fuzztest::InRange(1, 20),                                                           // avgWindowSize
        fuzztest::InRange(-1.0, 10.0),                                                      // brightnessThreshold
        fuzztest::VectorOf(fuzztest::InRange(0, MAX_STEP_PIXELS)).WithSize(MAX_FUZZ_STEPS)  // pixelCountsPerStep
    );
