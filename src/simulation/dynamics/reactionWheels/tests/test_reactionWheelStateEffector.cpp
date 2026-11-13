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

//! addReactionWheel must refuse wheels beyond RW_EFF_CNT so the fixed-size
//! output/command arrays cannot overflow.
TEST(ReactionWheelStateEffector, addReactionWheelCapsAtRwEffCnt) {
    ReactionWheelStateEffector rw;
    std::vector<RWConfigMsgPayload> configs(RW_EFF_CNT + 5, makeRW(BalancedWheels, Eigen::Vector3d::UnitX()));
    for (auto &cfg : configs) { rw.addReactionWheel(&cfg); }
    EXPECT_EQ(rw.ReactionWheelData.size(), static_cast<size_t>(RW_EFF_CNT));
    EXPECT_EQ(rw.rwOutMsgs.size(), static_cast<size_t>(RW_EFF_CNT));
}

//! With a mixed configuration (numRWJitter < numRW) each jitter wheel's
//! theta must map to its own thetasState slot, and non-jitter wheels report 0.
//! Theta state is set to known, distinct values so the mapping is unambiguous.
TEST(ReactionWheelStateEffector, thetaOutputMappingMixedConfig) {
    ReactionWheelStateEffector rw;

    std::vector<RWConfigMsgPayload> cfgs;
    cfgs.reserve(3);
    cfgs.push_back(makeRW(BalancedWheels, Eigen::Vector3d::UnitX()));      // wheel 0: no theta
    cfgs.push_back(makeRW(JitterSimple, Eigen::Vector3d::UnitY()));        // wheel 1: theta slot 0
    cfgs.push_back(makeRW(JitterFullyCoupled, Eigen::Vector3d::UnitZ()));  // wheel 2: theta slot 1
    for (auto &c : cfgs) { rw.addReactionWheel(&c); }

    DynParamManager dpm;
    rwtest::linkAndRegister(dpm, rw);

    ASSERT_EQ(rw.numRW, 3u);
    ASSERT_EQ(rw.numRWJitter, 2u);

    // Known, distinct state values: theta slots [0.11, 0.22], speeds [10, 20, 30].
    Eigen::MatrixXd thetas(2, 1);
    thetas << 0.11, 0.22;
    dpm.getStateObject(rw.nameOfReactionWheelThetasState)->setState(thetas);
    Eigen::MatrixXd omegas(3, 1);
    omegas << 10.0, 20.0, 30.0;
    dpm.getStateObject(rw.nameOfReactionWheelOmegasState)->setState(omegas);

    rw.writeOutputStateMessages(0);

    // Balanced wheel (0) has no theta DOF -> stays 0; jitter wheels map to their
    // own theta slots, NOT the wheel index.
    EXPECT_DOUBLE_EQ(rw.rwSpeedMsgBuffer.wheelThetas[0], 0.0);
    EXPECT_DOUBLE_EQ(rw.rwSpeedMsgBuffer.wheelThetas[1], 0.11);
    EXPECT_DOUBLE_EQ(rw.rwSpeedMsgBuffer.wheelThetas[2], 0.22);
    EXPECT_DOUBLE_EQ(rw.rwSpeedMsgBuffer.wheelSpeeds[0], 10.0);
    EXPECT_DOUBLE_EQ(rw.rwSpeedMsgBuffer.wheelSpeeds[1], 20.0);
    EXPECT_DOUBLE_EQ(rw.rwSpeedMsgBuffer.wheelSpeeds[2], 30.0);

    // Same mapping must hold for the per-wheel config-log path.
    rw.WriteOutputMessages(0);
    EXPECT_DOUBLE_EQ(rw.ReactionWheelData[0]->theta, 0.0);
    EXPECT_DOUBLE_EQ(rw.ReactionWheelData[1]->theta, 0.11);
    EXPECT_DOUBLE_EQ(rw.ReactionWheelData[2]->theta, 0.22);
}

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
