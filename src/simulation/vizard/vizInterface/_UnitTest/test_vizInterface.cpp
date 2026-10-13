// SPDX-License-Identifier: ISC
// Copyright (c) 2026, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

#include "fakeVizard.hpp"
#include "vizTestHelpers.hpp"

#include <gtest/gtest.h>

using viztest::FakeVizard;
using viztest::parseFrame;
using viztest::Request;
using viztest::VizHarness;

//! In live stream mode, reset sends two PING requests. Each later step sends one SIM_UPDATE
//! request with two empty frames before the serialized message. The step at time zero sends nothing.
TEST(VizInterfaceLiveStream, sendsHandshakeThenOneUpdatePerStep) {
    FakeVizard vizard;
    VizHarness harness;
    harness.viz.liveStream = true;
    harness.viz.comPortNumber = vizard.getPort();
    harness.addSpacecraft({"sat", 2, 2, 1});
    harness.addPlanet("earth", true);

    harness.run(3);
    std::vector<Request> const& requests = vizard.stop();

    ASSERT_EQ(requests.size(), 4u);
    EXPECT_EQ(requests[0], Request{"PING"});
    EXPECT_EQ(requests[1], Request{"PING"});
    for (size_t k = 2; k < 4; k++) {
        ASSERT_EQ(requests[k].size(), 4u);
        EXPECT_EQ(requests[k][0], "SIM_UPDATE");
        EXPECT_EQ(requests[k][1], "");
        EXPECT_EQ(requests[k][2], "");
    }
    auto const first = parseFrame(requests[2][3]);
    auto const second = parseFrame(requests[3][3]);
    EXPECT_EQ(first.currenttime().framenumber(), 1);
    EXPECT_EQ(second.currenttime().framenumber(), 2);
    EXPECT_TRUE(first.has_settings());
    EXPECT_FALSE(second.has_settings());
    EXPECT_TRUE(first.has_epoch());
    EXPECT_FALSE(second.has_epoch());
    ASSERT_EQ(second.spacecraft_size(), 1);
    EXPECT_EQ(second.spacecraft(0).reactionwheels_size(), 2);
    EXPECT_EQ(second.spacecraft(0).thrusters_size(), 2);
    EXPECT_EQ(second.spacecraft(0).css_size(), 1);
    EXPECT_EQ(second.celestialbodies_size(), 1);
}
