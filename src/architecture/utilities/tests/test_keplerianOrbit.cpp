#include <gtest/gtest.h>

#include "architecture/utilities/keplerianOrbit.h"
#include "architecture/utilities/orbitalMotion.h"

#include <cmath>
#include <numbers>

constexpr double kScalarTolerance = 1e-12;
constexpr double kVectorTolerance = 1e-12;

TEST(KeplerianOrbit, BasicBehavior) {
    KeplerianOrbit orb;
    EXPECT_DOUBLE_EQ(orb.a(), 100000.0);

    ClassicElements oe = orb.oe();
    KeplerianOrbit orb2(oe, MU_EARTH);
    EXPECT_TRUE(orb2.r_BP_P().isApprox(orb.r_BP_P(), kVectorTolerance));

    KeplerianOrbit orb3(orb2);
    EXPECT_TRUE(orb2.v_BP_P().isApprox(orb3.v_BP_P(), kVectorTolerance));

    KeplerianOrbit orb4 = orb3;
    EXPECT_TRUE(orb3.r_BP_P().isApprox(orb4.r_BP_P(), kVectorTolerance));

    orb3.set_f(0.0);
    const double initial_radius = orb3.r();
    orb3.set_f(1.0);
    EXPECT_GT(std::abs(orb3.r() - initial_radius), kScalarTolerance);

    orb3.set_f(0.0);
    EXPECT_NEAR(orb3.r(), initial_radius, kScalarTolerance);

    const double expected_mean_motion = std::sqrt(MU_EARTH / std::pow(orb3.a(), 3));
    EXPECT_NEAR(orb3.n(), expected_mean_motion, kScalarTolerance);

    const double expected_period = 2.0 * std::numbers::pi / expected_mean_motion;
    EXPECT_NEAR(orb3.P(), expected_period, kScalarTolerance);

    const double expected_energy = -MU_EARTH / (2.0 * orb3.a());
    EXPECT_NEAR(orb3.Energy(), expected_energy, kScalarTolerance);

    ClassicElements expected_elements = orb3.oe();
    double expected_r[3] = {};
    double expected_v[3] = {};
    elem2rv(MU_EARTH, &expected_elements, expected_r, expected_v);

    Eigen::Vector3d expected_position(expected_r[0], expected_r[1], expected_r[2]);
    Eigen::Vector3d expected_velocity(expected_v[0], expected_v[1], expected_v[2]);
    EXPECT_LT((orb3.r_BP_P() - expected_position).norm(), kVectorTolerance);
    EXPECT_LT((orb3.v_BP_P() - expected_velocity).norm(), kVectorTolerance);
}
