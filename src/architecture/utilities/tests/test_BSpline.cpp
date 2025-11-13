#include <gtest/gtest.h>

#include "architecture/utilities/BSpline.h"

#include <initializer_list>
#include <tuple>

using BSplineParams = std::tuple<int, bool, bool, double>;

Eigen::VectorXd makeVector(std::initializer_list<double> values) {
    Eigen::VectorXd vec(values.size());
    Eigen::Index index = 0;
    for (double value : values) {
        vec(index++) = value;
    }
    return vec;
}

class BSplineInterpolation : public ::testing::TestWithParam<BSplineParams> {};

TEST_P(BSplineInterpolation, HitsWaypointsAndDerivatives) {
    const auto params = GetParam();
    const int order = std::get<0>(params);
    const bool set_first_derivatives = std::get<1>(params);
    const bool set_second_derivatives = std::get<2>(params);
    const double tolerance = std::get<3>(params);

    Eigen::VectorXd X1 = makeVector({0.0, 1.0, 2.0, 3.0, 4.0, 5.0, 6.0});
    Eigen::VectorXd X2 = makeVector({5.0, 4.0, 3.0, 2.0, 1.0, 0.0, 1.0});
    Eigen::VectorXd X3 = makeVector({3.0, 2.0, 1.0, 2.0, 3.0, 4.0, 5.0});

    InputDataSet input(X1, X2, X3);
    input.setT(makeVector({0.0, 2.0, 3.0, 5.0, 7.0, 8.0, 10.0}));

    if (set_first_derivatives) {
        input.setXDot_0(Eigen::Vector3d::Zero());
        input.setXDot_N(Eigen::Vector3d::Zero());
    }

    if (set_second_derivatives) {
        input.setXDDot_0(Eigen::Vector3d::Zero());
        input.setXDDot_N((Eigen::Vector3d() << 0.2, 0.0, 0.0).finished());
    }

    OutputDataSet output;
    interpolate(input, 101, order, &output);

    ASSERT_EQ(output.T.size(), output.X1.size());
    ASSERT_EQ(output.T.size(), output.X2.size());
    ASSERT_EQ(output.T.size(), output.X3.size());

    const Eigen::Index num_inputs = input.T.size();
    const Eigen::Index num_outputs = output.T.size();

    for (Eigen::Index j = 0; j < num_inputs; ++j) {
        bool matched = false;
        for (Eigen::Index i = 0; i < num_outputs; ++i) {
            if (std::abs(output.T(i) - input.T(j)) < tolerance) {
                matched = true;
                EXPECT_NEAR(output.X1(i), X1(j), tolerance)
                    << "Failed coordinate X1 at t = " << input.T(j) << " with order " << order;
                EXPECT_NEAR(output.X2(i), X2(j), tolerance)
                    << "Failed coordinate X2 at t = " << input.T(j) << " with order " << order;
                EXPECT_NEAR(output.X3(i), X3(j), tolerance)
                    << "Failed coordinate X3 at t = " << input.T(j) << " with order " << order;
            }
        }
        EXPECT_TRUE(matched) << "No matching sample found at original waypoint t = " << input.T(j);
    }

    if (set_first_derivatives) {
        EXPECT_NEAR(output.XD1(0), input.XDot_0(0), tolerance);
        EXPECT_NEAR(output.XD2(0), input.XDot_0(1), tolerance);
        EXPECT_NEAR(output.XD3(0), input.XDot_0(2), tolerance);
    }

    if (set_second_derivatives) {
        EXPECT_NEAR(output.XDD1(0), input.XDDot_0(0), tolerance);
        EXPECT_NEAR(output.XDD2(0), input.XDDot_0(1), tolerance);
        EXPECT_NEAR(output.XDD3(0), input.XDDot_0(2), tolerance);
    }
}

INSTANTIATE_TEST_SUITE_P(
    Utilities,
    BSplineInterpolation,
    ::testing::Combine(::testing::Values(5, 6), ::testing::Bool(), ::testing::Bool(), ::testing::Values(1e-6)));
