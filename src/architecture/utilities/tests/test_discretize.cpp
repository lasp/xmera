#include "architecture/utilities/discretize.h"
#include <Eigen/Dense>
#include <gtest/gtest.h>


TEST(Discretize, testRoundToZero) {
    Discretize discretizor = Discretize(3);
    discretizor.setRoundDirection(TO_ZERO);
    discretizor.setCarryError(false);

    Eigen::Vector3d LSBs;
    LSBs << 10.0, 10.0, 10.0;
    discretizor.setLSB(LSBs);

    Eigen::Vector3d expected;
    expected << 0, 10.0, 10.0;
    Eigen::Vector3d states;
    states << 0.1, 10.1, 11.1;
    states = discretizor.discretize(states);

    EXPECT_TRUE(states == expected);
}

TEST(Discretize, testRoundFromZero) {
    Discretize discretizor = Discretize(3);
    discretizor.setRoundDirection(FROM_ZERO);
    discretizor.setCarryError(false);

    Eigen::Vector3d LSBs;
    LSBs << 10.0, 10.0, 10.0;
    discretizor.setLSB(LSBs);

    Eigen::Vector3d states;
    states << 0.1, 10.1, 11.1;
    Eigen::Vector3d expected;
    expected << 10.0, 20.0, 20.0;
    states = discretizor.discretize(states);

    EXPECT_TRUE(states == expected);
}

TEST(Discretize, testRoundNear) {
    Discretize discretizor = Discretize(3);
    discretizor.setRoundDirection(NEAR);
    discretizor.setCarryError(false);

    Eigen::Vector3d LSBs;
    LSBs << 10.0, 10.0, 10.0;
    discretizor.setLSB(LSBs);

    Eigen::Vector3d states;
    states << 0.1, 10.1, 15.1;
    Eigen::Vector3d expected;
    expected << 0, 10, 20;
    states = discretizor.discretize(states);

    EXPECT_TRUE(states == expected);
}

TEST(Discretize, testRoundToZeroCarryError) {
    Discretize discretizor = Discretize(3);
    discretizor.setRoundDirection(TO_ZERO);
    discretizor.setCarryError(true);

    Eigen::Vector3d LSBs;
    LSBs << 10.0, 10.0, 10.0;
    discretizor.setLSB(LSBs);

    Eigen::Vector3d expected;
    expected << 0.0, 10.0, 10.0;
    Eigen::Vector3d states;
    states << 0.1, 10.1, 15;

    Eigen::Vector3d output;
    for(uint64_t i = 0; i < 2; i++){
        output = discretizor.discretize(states);
        EXPECT_TRUE(output == expected);
        expected << 0, 10, 20;
    }
}
