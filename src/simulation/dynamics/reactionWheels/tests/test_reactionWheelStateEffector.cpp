// SPDX-License-Identifier: ISC
// Copyright (c) 2026, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

#include "reactionWheelTestHelpers.hpp"
#include <architecture/msgPayloadDef/RWConfigMsgPayload.h>

#include <gtest/gtest.h>
#include <mission/parameters.h>
#include <simulation/dynamics/_GeneralModuleFiles/dynParamManager.h>
#include <simulation/dynamics/reactionWheels/reactionWheelStateEffector.h>

#include <Eigen/Dense>
#include <vector>

using rwtest::makeRW;

//! Sanity: all-jitter configuration (numRWJitter == numRW) maps theta slot i to
//! wheel i, and an all-balanced configuration reports zero thetas.
TEST(ReactionWheelStateEffector, thetaOutputMappingSingleModel) {
    // All jitter.
    {
        ReactionWheelStateEffector rw;
        std::vector<RWConfigMsgPayload> cfgs(3, makeRW(JitterSimple, Eigen::Vector3d::UnitX()));
        for (auto &c : cfgs) { rw.addReactionWheel(&c); }
        DynParamManager dpm;
        rwtest::linkAndRegister(dpm, rw);
        Eigen::MatrixXd thetas(3, 1);
        thetas << 0.3, 0.6, 0.9;
        dpm.getStateObject(rw.nameOfReactionWheelThetasState)->setState(thetas);
        rw.writeOutputStateMessages(0);
        EXPECT_DOUBLE_EQ(rw.rwSpeedMsgBuffer.wheelThetas[0], 0.3);
        EXPECT_DOUBLE_EQ(rw.rwSpeedMsgBuffer.wheelThetas[1], 0.6);
        EXPECT_DOUBLE_EQ(rw.rwSpeedMsgBuffer.wheelThetas[2], 0.9);
    }
    // All balanced -> no theta state registered, thetas stay 0.
    {
        ReactionWheelStateEffector rw;
        std::vector<RWConfigMsgPayload> cfgs(3, makeRW(BalancedWheels, Eigen::Vector3d::UnitX()));
        for (auto &c : cfgs) { rw.addReactionWheel(&c); }
        DynParamManager dpm;
        rwtest::linkAndRegister(dpm, rw);
        ASSERT_EQ(rw.numRWJitter, 0u);
        rw.writeOutputStateMessages(0);
        EXPECT_DOUBLE_EQ(rw.rwSpeedMsgBuffer.wheelThetas[0], 0.0);
        EXPECT_DOUBLE_EQ(rw.rwSpeedMsgBuffer.wheelThetas[1], 0.0);
        EXPECT_DOUBLE_EQ(rw.rwSpeedMsgBuffer.wheelThetas[2], 0.0);
    }
}
