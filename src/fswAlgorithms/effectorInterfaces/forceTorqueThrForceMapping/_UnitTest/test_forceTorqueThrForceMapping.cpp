// SPDX-License-Identifier: ISC
// Copyright (c) 2026, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

#include "forceTorqueThrForceMappingAlgorithm.h"

#include <gtest/gtest.h>

#include <algorithm>
#include <cstddef>
#include <Eigen/Geometry>
#include <Eigen/SVD>
#include <string>
#include <vector>

namespace {
    constexpr double tolerance = 1e-12;
    constexpr double maxThrust = 3.0;  // [N]
    Eigen::Vector3d const CoM_B{0.1, 0.1, 0.1};

    //! One thruster layout and one force and torque request.
    struct MappingCase {
        std::string name;
        std::vector<Eigen::Vector3d> locations;
        std::vector<Eigen::Vector3d> directions;
        Eigen::Vector3d torque;
        Eigen::Vector3d force;
    };

    //! Eight thrusters at the corners of a box. No thruster points along z.
    std::vector<Eigen::Vector3d> const boxLocations{
        {-0.86360, -0.82550, 1.79070},
        {-0.82550, -0.86360, 1.79070},
        {0.82550, 0.86360, 1.79070},
        {0.86360, 0.82550, 1.79070},
        {-0.86360, -0.82550, -1.79070},
        {-0.82550, -0.86360, -1.79070},
        {0.82550, 0.86360, -1.79070},
        {0.86360, 0.82550, -1.79070}
    };
    std::vector<Eigen::Vector3d> const boxDirections{
        {1.0, 0.0, 0.0},
        {0.0, 1.0, 0.0},
        {0.0, -1.0, 0.0},
        {-1.0, 0.0, 0.0},
        {1.0, 0.0, 0.0},
        {0.0, 1.0, 0.0},
        {0.0, -1.0, 0.0},
        {-1.0, 0.0, 0.0}
    };

    //! Twelve thrusters in four groups of three. Thrusters point along all axes.
    std::vector<Eigen::Vector3d> const balancedLocations{
        {-1, -1, 1},
        {-1, -1, 1},
        {-1, -1, 1},
        {1, 1, 1},
        {1, 1, 1},
        {1, 1, 1},
        {1, 1, -1},
        {1, 1, -1},
        {1, 1, -1},
        {-1, -1, -1},
        {-1, -1, -1},
        {-1, -1, -1}
    };
    std::vector<Eigen::Vector3d> const balancedDirections{
        {1.0, 0.0, 0.0},
        {0.0, 1.0, 0.0},
        {0.0, 0.0, -1.0},
        {0.0, 0.0, -1.0},
        {0.0, -1.0, 0.0},
        {-1.0, 0.0, 0.0},
        {0.0, -1.0, 0.0},
        {-1.0, 0.0, 0.0},
        {0.0, 0.0, 1.0},
        {1.0, 0.0, 0.0},
        {0.0, 1.0, 0.0},
        {0.0, 0.0, 1.0}
    };

    //! Six thrusters with general locations and directions.
    std::vector<Eigen::Vector3d> const generalLocations{
        {0.5, -0.3, 1.2},
        {-0.7, 0.4, 0.9},
        {1.1, 0.2, -0.6},
        {-0.2, -1.0, 0.3},
        {0.8, 0.9, -1.1},
        {-1.2, 0.6, -0.4}
    };
    std::vector<Eigen::Vector3d> const generalDirections{
        {0.6, 0.8, 0.0},
        {0.0, 0.6, -0.8},
        {-0.8, 0.0, 0.6},
        {0.48, 0.6, 0.64},
        {-0.6, -0.48, 0.64},
        {0.64, -0.6, -0.48}
    };

    VehicleConfigMsgPayload makeVehicleConfig() {
        VehicleConfigMsgPayload vehConfig{};
        for (std::size_t i = 0; i < 3; ++i) { vehConfig.CoM_B[i] = CoM_B[static_cast<Eigen::Index>(i)]; }
        return vehConfig;
    }

    //! Writes the thrusters into a configuration payload. The caller makes sure they fit in MAX_EFF_CNT.
    THRArrayConfigMsgPayload
    makeThrusterConfig(std::vector<Eigen::Vector3d> const &locations, std::vector<Eigen::Vector3d> const &directions) {
        THRArrayConfigMsgPayload thrConfig{};
        thrConfig.numThrusters = static_cast<uint32_t>(locations.size());
        for (std::size_t i = 0; i < locations.size(); ++i) {
            for (Eigen::Index j = 0; j < 3; ++j) {
                thrConfig.thrusters[i].rThrust_B[j] = locations[i][j];
                thrConfig.thrusters[i].tHatThrust_B[j] = directions[i][j];
            }
            thrConfig.thrusters[i].maxThrust = maxThrust;
        }
        return thrConfig;
    }

    //! Minimum-norm thruster forces from an SVD, shifted so that the smallest force is zero.
    Eigen::VectorXd computeTruth(MappingCase const &c) {
        auto const n = static_cast<Eigen::Index>(c.locations.size());
        Eigen::MatrixXd DG(6, n);
        for (Eigen::Index i = 0; i < n; ++i) {
            auto const k = static_cast<std::size_t>(i);
            DG.col(i) << (c.locations[k] - CoM_B).cross(c.directions[k]), c.directions[k];
        }
        Eigen::Vector<double, 6> forceTorque;
        forceTorque << c.torque, c.force;

        std::vector<Eigen::Index> keptRows;
        for (Eigen::Index i = 0; i < 6; ++i) {
            if ((DG.row(i).array().abs() > 1e-7).any()) { keptRows.push_back(i); }
        }
        Eigen::MatrixXd DGKept(static_cast<Eigen::Index>(keptRows.size()), n);
        Eigen::VectorXd forceTorqueKept(static_cast<Eigen::Index>(keptRows.size()));
        for (std::size_t i = 0; i < keptRows.size(); ++i) {
            DGKept.row(static_cast<Eigen::Index>(i)) = DG.row(keptRows[i]);
            forceTorqueKept[static_cast<Eigen::Index>(i)] = forceTorque[keptRows[i]];
        }

        Eigen::VectorXd const force =
            DGKept.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(forceTorqueKept);
        return force.array() - force.minCoeff();
    }

    THRArrayCmdForceMsgPayload runAlgorithm(MappingCase const &c) {
        ForceTorqueThrForceMappingAlgorithm algorithm{};
        VehicleConfigMsgPayload vehConfig = makeVehicleConfig();
        THRArrayConfigMsgPayload thrConfig = makeThrusterConfig(c.locations, c.directions);
        algorithm.reset(vehConfig, thrConfig);

        CmdTorqueBodyMsgPayload cmdTorque{};
        CmdForceBodyMsgPayload cmdForce{};
        for (Eigen::Index j = 0; j < 3; ++j) {
            cmdTorque.torqueRequestBody[j] = c.torque[j];
            cmdForce.forceRequestBody[j] = c.force[j];
        }
        return algorithm.update(cmdTorque, cmdForce);
    }
}  // namespace

class ForceTorqueThrForceMappingCase : public ::testing::TestWithParam<MappingCase> {};

//! The thruster forces agree with an independent SVD solution. All forces are zero or more, the
//! smallest force is zero, and the entries after the last thruster are zero.
TEST_P(ForceTorqueThrForceMappingCase, matchesMinimumNormSolution) {
    MappingCase const &c = GetParam();
    if (c.locations.size() > MAX_EFF_CNT) { GTEST_SKIP() << "MAX_EFF_CNT is less than the number of thrusters"; }

    THRArrayCmdForceMsgPayload const out = runAlgorithm(c);
    Eigen::VectorXd const truth = computeTruth(c);

    double minForce = out.thrForce[0];
    for (std::size_t i = 0; i < c.locations.size(); ++i) {
        EXPECT_NEAR(out.thrForce[i], truth[static_cast<Eigen::Index>(i)], tolerance) << "thruster " << i;
        EXPECT_GE(out.thrForce[i], 0.0) << "thruster " << i;
        minForce = std::min(minForce, out.thrForce[i]);
    }
    EXPECT_DOUBLE_EQ(minForce, 0.0);
    for (std::size_t i = c.locations.size(); i < MAX_EFF_CNT; ++i) { EXPECT_DOUBLE_EQ(out.thrForce[i], 0.0); }
}

INSTANTIATE_TEST_SUITE_P(
    FixedLayouts,
    ForceTorqueThrForceMappingCase,
    ::testing::Values(
        // No thruster points along z, so the z force row of DG is zero and the algorithm removes it.
        MappingCase{"boxTorqueAndForce", boxLocations, boxDirections, {0.4, 0.2, 0.4}, {0.9, 1.1, 0.0}},
        MappingCase{"boxForceOnly", boxLocations, boxDirections, {0.0, 0.0, 0.0}, {0.9, 1.1, 0.0}},
        MappingCase{"balancedForceOnly", balancedLocations, balancedDirections, {0.0, 0.0, 0.0}, {0.9, 1.1, 1.0}},
        MappingCase{"generalTorqueAndForce", generalLocations, generalDirections, {0.3, -0.2, 0.5}, {0.4, 0.7, -0.1}}
    ),
    [](::testing::TestParamInfo<MappingCase> const &info) { return info.param.name; }
);
