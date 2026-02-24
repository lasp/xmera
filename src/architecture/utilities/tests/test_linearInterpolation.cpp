// SPDX-License-Identifier: ISC
// Copyright (c) 2015, Autonomous Vehicle System Lab, University of Colorado at Boulder
// Copyright (c) 2024, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

#include "architecture/utilities/linearInterpolation.hpp"
#include <gtest/gtest.h>
#include <random>

std::random_device rd;
std::default_random_engine generator(rd());
std::uniform_real_distribution<double> valueDistribution(-10, 10);
std::uniform_real_distribution<double> boundDistribution(0, 2);

TEST(LinearInterpolationTest, HandlesNormalInputs) {
    double x = valueDistribution(generator);
    double x1 = x - boundDistribution(generator);
    double x2 = x + boundDistribution(generator);

    double yUnused = valueDistribution(generator);
    double y1 = yUnused - boundDistribution(generator);
    double y2 = yUnused + boundDistribution(generator);

    // Linearly interpolate to solve for y
    double y = y1 * (x2 - x) / (x2 - x1) + y2 * (x - x1) / (x2 - x1);

    EXPECT_EQ(linearInterpolation(x1, x2, y1, y2, x), y);
}
