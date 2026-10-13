// SPDX-License-Identifier: ISC
// Copyright (c) 2026, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

#include "fakeVizard.hpp"
#include "vizTestHelpers.hpp"

#include <gtest/gtest.h>

using viztest::commands;
using viztest::FakeVizard;
using viztest::oneSecond;
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

//! In opNav mode 2, each step with an image sends SIM_UPDATE, then REQUEST_IMAGE_<id>, then PING.
//! The image goes to the image output message of the camera.
TEST(VizInterfaceOpNav, modeTwoRequestsImageThenPings) {
    FakeVizard vizard;
    vizard.setImage("PIXELS");
    VizHarness harness;
    harness.viz.opNavMode = 2;
    harness.viz.comPortNumber = vizard.getPort();
    harness.addSpacecraft({"sat", 0, 0, 0});
    harness.addCamera(1, oneSecond);
    ReadFunctor<CameraImageMsgPayload> imageReader = harness.viz.opnavImageOutMsgs[0]->addSubscriber();

    harness.run(3);
    std::vector<Request> const& requests = vizard.stop();

    std::vector<std::string> const expected{
        "PING", "PING", "SIM_UPDATE", "REQUEST_IMAGE_1", "PING", "SIM_UPDATE", "REQUEST_IMAGE_1", "PING"};
    EXPECT_EQ(commands(requests), expected);
    CameraImageMsgPayload const image = imageReader();
    EXPECT_EQ(image.valid, 1);
    EXPECT_EQ(image.cameraID, 1);
    EXPECT_EQ(image.timeTag, 2 * oneSecond);
    ASSERT_EQ(image.imageBufferLength, 6);
    EXPECT_EQ(std::string(static_cast<char const*>(image.imagePointer), 6), "PIXELS");
}

//! In opNav mode 1, each step sends SIM_UPDATE, then REQUEST_IMAGE_<id> when an image is due.
//! This mode does not send PING after the image.
TEST(VizInterfaceOpNav, modeOneRequestsImageWithoutPing) {
    FakeVizard vizard;
    VizHarness harness;
    harness.viz.opNavMode = 1;
    harness.viz.comPortNumber = vizard.getPort();
    harness.addSpacecraft({"sat", 0, 0, 0});
    harness.addCamera(3, oneSecond);

    harness.run(3);
    std::vector<Request> const& requests = vizard.stop();

    std::vector<std::string> const expected{
        "PING", "PING", "SIM_UPDATE", "REQUEST_IMAGE_3", "SIM_UPDATE", "REQUEST_IMAGE_3"};
    EXPECT_EQ(commands(requests), expected);
}
