// SPDX-License-Identifier: ISC
// Copyright (c) 2015, Autonomous Vehicle System Lab, University of Colorado at Boulder
// Copyright (c) 2024, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

#include "architecture/utilities/bilinearInterpolation.hpp"
#include <gtest/gtest.h>
#include <random>

std::random_device rd;
std::default_random_engine generator(rd());
std::uniform_real_distribution<double> valueDistribution(-10, 10);
std::uniform_real_distribution<double> boundDistribution(0, 2);

TEST(BilinearInterpolationTest, HandlesNormalInputs) {
    double x = valueDistribution(generator);
    double x1 = x - boundDistribution(generator);
    double x2 = x + boundDistribution(generator);

    double y = valueDistribution(generator);
    double y1 = y - boundDistribution(generator);
    double y2 = y + boundDistribution(generator);

    double z11 = valueDistribution(generator);
    double z12 = valueDistribution(generator);
    double z21 = valueDistribution(generator);
    double z22 = valueDistribution(generator);

    // Bilinearly interpolate to solve for z
    double z =
        1 / ((x2 - x1) * (y2 - y1)) *
        (z11 * (x2 - x) * (y2 - y) + z21 * (x - x1) * (y2 - y) + z12 * (x2 - x) * (y - y1) + z22 * (x - x1) * (y - y1));

    EXPECT_EQ(bilinearInterpolation(x1, x2, y1, y2, z11, z12, z21, z22, x, y), z);
}
