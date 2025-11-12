#include "architecture/utilities/saturate.h"
#include "architecture/utilities/linearAlgebra.h"
#include <gtest/gtest.h>


TEST(Saturate, testSaturate) {
    Eigen::Vector3d states;
    states << -555, 1.27, 5000000.;
    auto saturator = Saturate(3);
    Eigen::MatrixXd bounds;
    bounds.resize(3,2);
    bounds << -400., 0, 5, 10, -1, 5000001;
    saturator.setBounds(bounds);
    states = saturator.saturate(states);
    Eigen::Vector3d expected;
    expected << -400, 5, 5000000;
    EXPECT_TRUE(states == expected);
}
