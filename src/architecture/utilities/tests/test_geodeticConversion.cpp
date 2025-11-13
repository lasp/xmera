#include "architecture/utilities/geodeticConversion.h"
#include "architecture/utilities/linearAlgebra.h"
#include <gtest/gtest.h>

TEST(GeodeticConversion, testPCI2PCPF) {
    Eigen::Vector3d pciPosition;
    pciPosition << 1., 2., 3.;

    double J20002Pfix[3][3];
    m33Set(1, 0, 0, 0, 0, 1, 0, -1, 0, J20002Pfix);
    Eigen::Vector3d ans = PCI2PCPF(pciPosition, J20002Pfix);

    Eigen::Vector3d expected;
    expected << 1, 3, -2;
    EXPECT_TRUE(ans == expected);
}

TEST(GeodeticConversion, testPCPF2PCI) {
    Eigen::Vector3d pcpfPosition;
    pcpfPosition << 1., 2., 3.;

    double J20002Pfix[3][3];
    m33Set(1, 0, 0, 0, 0, 1, 0, -1, 0, J20002Pfix);
    Eigen::Vector3d ans = PCPF2PCI(pcpfPosition, J20002Pfix);

    Eigen::Vector3d expected;
    expected << 1, -3, 2;
    EXPECT_TRUE(ans == expected);
}

TEST(GeodeticConversion, testLLA2PCPF) {
    Eigen::Vector3d llaPosition;
    llaPosition << 0.6935805104270613, 1.832425562445269, 1596.668;

    Eigen::Vector3d ans = LLA2PCPF(llaPosition, 6378.1363E3, 6356.7523E3);

    Eigen::Vector3d expected;
    expected << -1270640.01, 4745322.63, 4056784.62;
    EXPECT_LT((expected - ans).norm(), 100.0);
}

TEST(GeodeticConversion, testPCPF2LLA) {
    Eigen::Vector3d pcpfPosition;
    pcpfPosition << -1270640.01, 4745322.63, 4056784.62;

    Eigen::Vector3d ans = PCPF2LLA(pcpfPosition, 6378.1363E3, 6356.7523E3);

    Eigen::Vector3d llaPosition;
    llaPosition << 0.6935805104270613, 1.832425562445269, 1596.668;

    EXPECT_LT(std::abs(llaPosition[0] - ans[0]), 0.0001);
    EXPECT_LT(std::abs(llaPosition[1] - ans[1]), 0.0001);
    EXPECT_LT(std::abs(llaPosition[2] - ans[2]), 100);
}
