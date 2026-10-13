// SPDX-License-Identifier: ISC
// Copyright (c) 2026, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

#include "fakeVizard.hpp"
#include "vizTestHelpers.hpp"

#include <gtest/gtest.h>

using viztest::commands;
using viztest::FakeVizard;
using viztest::oneSecond;
using viztest::parseFrame;
using viztest::readFrames;
using viztest::Request;
using viztest::TempFile;
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

//! In file mode, each step after time zero adds one length-delimited frame to the file.
//! The first frame has the settings and the epoch. Each frame has the current input states.
TEST(VizInterfaceFile, writesOneDelimitedFramePerStep) {
    TempFile file("vizFrames");
    {
        VizHarness harness;
        harness.viz.saveFile = true;
        harness.viz.protoFilename = file.string();
        harness.addSpacecraft({"sat", 1, 1, 1});
        harness.addPlanet("earth", true);
        harness.addPlanet("moon", false);
        harness.run(3);
    }

    auto const frames = readFrames(file.string());

    ASSERT_EQ(frames.size(), 2u);
    EXPECT_EQ(frames[0].currenttime().framenumber(), 1);
    EXPECT_EQ(frames[1].currenttime().framenumber(), 2);
    EXPECT_DOUBLE_EQ(frames[1].currenttime().simtimeelapsed(), 2.0 * oneSecond);
    EXPECT_TRUE(frames[0].has_settings());
    EXPECT_FALSE(frames[1].has_settings());
    ASSERT_EQ(frames[1].spacecraft_size(), 1);
    auto const& sat = frames[1].spacecraft(0);
    EXPECT_EQ(sat.spacecraftname(), "sat");
    EXPECT_DOUBLE_EQ(sat.position(0), 7.0e6);
    EXPECT_DOUBLE_EQ(sat.rotation(2), 0.2);
    ASSERT_EQ(sat.reactionwheels_size(), 1);
    EXPECT_DOUBLE_EQ(sat.reactionwheels(0).wheelspeed(), 200.0);
    ASSERT_EQ(frames[1].celestialbodies_size(), 2);
    EXPECT_EQ(frames[1].celestialbodies(1).bodyname(), "moon");
    EXPECT_DOUBLE_EQ(frames[1].celestialbodies(1).rotation(0), 1.0);
    EXPECT_DOUBLE_EQ(frames[1].celestialbodies(1).position(0), 0.0);
}

//! With a saved file and opNav mode 2, each step still writes a complete frame to the file.
TEST(VizInterfaceFile, opNavModeTwoWritesCompleteFrames) {
    TempFile file("vizOpNavFrames");
    {
        FakeVizard vizard;
        VizHarness harness;
        harness.viz.opNavMode = 2;
        harness.viz.saveFile = true;
        harness.viz.protoFilename = file.string();
        harness.viz.comPortNumber = vizard.getPort();
        harness.addSpacecraft({"sat", 0, 0, 0});
        harness.addCamera(1, oneSecond);
        harness.run(4);
        vizard.stop();
    }

    auto const frames = readFrames(file.string());

    ASSERT_EQ(frames.size(), 3u);
    EXPECT_EQ(frames[2].currenttime().framenumber(), 3);
    EXPECT_EQ(frames[2].cameras_size(), 1);
}

//! A second reset closes the first connection and makes the PING handshake again.
TEST(VizInterfaceLiveStream, resetAgainReconnects) {
    FakeVizard vizard;
    VizHarness harness;
    harness.viz.liveStream = true;
    harness.viz.comPortNumber = vizard.getPort();
    harness.addSpacecraft({"sat", 0, 0, 0});

    harness.run(2);
    harness.run(2);
    std::vector<Request> const& requests = vizard.stop();

    std::vector<std::string> const expected{"PING", "PING", "SIM_UPDATE", "PING", "PING", "SIM_UPDATE"};
    EXPECT_EQ(commands(requests), expected);
}

//! When the length frame claims more bytes than the image frame has, the image message is not valid.
TEST(VizInterfaceOpNav, rejectsImageLengthLargerThanFrame) {
    FakeVizard vizard;
    vizard.setImage("SHORT");
    vizard.setImageLengthOverride(1'000'000);
    VizHarness harness;
    harness.viz.opNavMode = 2;
    harness.viz.comPortNumber = vizard.getPort();
    harness.addSpacecraft({"sat", 0, 0, 0});
    harness.addCamera(1, oneSecond);
    ReadFunctor<CameraImageMsgPayload> imageReader = harness.viz.opnavImageOutMsgs[0]->addSubscriber();

    harness.run(2);
    vizard.stop();

    CameraImageMsgPayload const image = imageReader();
    EXPECT_EQ(image.valid, 0);
    EXPECT_EQ(image.imageBufferLength, 0);
    EXPECT_EQ(image.imagePointer, nullptr);
}

//! A camera with a render rate of zero does not request images and does not divide by zero.
//! Without an image request, the socket still waits for the SIM_UPDATE reply, thus the
//! keep-alive PING is not sent.
TEST(VizInterfaceOpNav, zeroRenderRateRequestsNoImage) {
    FakeVizard vizard;
    VizHarness harness;
    harness.viz.opNavMode = 2;
    harness.viz.comPortNumber = vizard.getPort();
    harness.addSpacecraft({"sat", 0, 0, 0});
    harness.addCamera(1, 0);

    harness.run(2);
    std::vector<Request> const& requests = vizard.stop();

    std::vector<std::string> const expected{"PING", "PING", "SIM_UPDATE"};
    EXPECT_EQ(commands(requests), expected);
}

//! Reset throws when a body name does not fit in the spice message name.
TEST(VizInterfaceReset, throwsForBodyNameTooLong) {
    VizHarness harness;
    harness.addSpacecraft({"sat", 0, 0, 0});
    harness.addPlanet(std::string(MAX_BODY_NAME_LENGTH, 'x'), false);

    EXPECT_THROW(harness.viz.reset(0), std::invalid_argument);
}
