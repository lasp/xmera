// SPDX-License-Identifier: ISC
// Copyright (c) 2026, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

#include "cielimInterface.h"
#include "fakeCielim.hpp"

#include <google/protobuf/io/zero_copy_stream_impl.h>
#include <google/protobuf/util/delimited_message_util.h>
#include <gtest/gtest.h>
#include <unistd.h>

#include <filesystem>
#include <fstream>
#include <string>
#include <vector>

using cielimtest::FakeCielim;
using cielimtest::Request;

namespace {
    constexpr uint64_t oneSecond = 1'000'000'000;

    //! Owns a CielimInterface and the input messages that it reads.
    class CielimHarness {
    public:
        CielimHarness() {
            this->cielim.spacecraftMessage.subscribeTo(&this->spacecraftMsg);
        }

        //! Connect a camera that takes one image each renderRate nanoseconds.
        void addCamera(int cameraId, uint64_t renderRate) {
            CameraModelMsgPayload camera = {};
            camera.cameraId = cameraId;
            camera.isOn = true;
            camera.renderRate = renderRate;
            this->cameraMsg.write(camera, 0, 0);
            this->cielim.cameraModelMessage.subscribeTo(&this->cameraMsg);
        }

        //! Reset the module, then run steps at 0, 1, ..., (numSteps - 1) seconds.
        void run(size_t numSteps) {
            this->cielim.reset(0);
            for (size_t k = 0; k < numSteps; k++) {
                uint64_t const t = k * oneSecond;
                SCStatesMsgPayload state = {};
                state.r_BN_N[0] = 7.0e6;
                this->spacecraftMsg.write(state, 0, t);
                this->cielim.updateState(t);
            }
        }

        CielimInterface cielim;

    private:
        Message<SCStatesMsgPayload> spacecraftMsg;
        Message<CameraModelMsgPayload> cameraMsg;
    };

    //! Read all length-delimited frames of a saved Cielim file.
    std::vector<cielimMessage::CielimMessage> readFrames(std::string const &path) {
        std::ifstream input(path, std::ios::binary);
        google::protobuf::io::IstreamInputStream stream(&input);
        std::vector<cielimMessage::CielimMessage> frames;
        while (true) {
            cielimMessage::CielimMessage frame;
            bool cleanEof = false;
            if (!google::protobuf::util::ParseDelimitedFromZeroCopyStream(&frame, &stream, &cleanEof)) { break; }
            frames.push_back(frame);
        }
        return frames;
    }

    //! Return the first frame of each request, in order.
    std::vector<std::string> commands(std::vector<Request> const &requests) {
        std::vector<std::string> names;
        for (auto const &request : requests) { names.push_back(request.front()); }
        return names;
    }
}  // namespace

//! In ALL_FRAMES mode, connect sends PING and INIT_SCENE. Each step with an image due sends
//! SIM_UPDATE, then a four-frame REQUEST_IMAGE, then PING. The image goes to the image output message.
TEST(CielimInterfaceLiveStream, allFramesRequestsImageEachStep) {
    FakeCielim renderer;
    renderer.setImage("PIXELS");
    CielimHarness harness;
    std::string port = renderer.getPort();
    harness.cielim.setPortNumber(port);
    harness.addCamera(2, oneSecond);
    ReadFunctor<CameraImageMsgPayload> imageReader = harness.cielim.imageOutMessage.addSubscriber();

    harness.run(3);
    std::vector<Request> const &requests = renderer.stop();

    std::vector<std::string> const expected{
        "PING",
        "INIT_SCENE",
        "SIM_UPDATE",
        "REQUEST_IMAGE",
        "PING",
        "SIM_UPDATE",
        "REQUEST_IMAGE",
        "PING",
    };
    EXPECT_EQ(commands(requests), expected);
    ASSERT_EQ(requests[3].size(), 4u);
    EXPECT_EQ(requests[3][1], "2");
    CameraImageMsgPayload const image = imageReader();
    EXPECT_EQ(image.valid, 1);
    EXPECT_EQ(image.timeTag, 2 * oneSecond);
    ASSERT_EQ(image.imageBufferLength, 6);
    EXPECT_EQ(std::string(static_cast<char const*>(image.imagePointer), 6), "PIXELS");
}

//! A camera with a render rate of zero does not request images and does not divide by zero.
TEST(CielimInterfaceLiveStream, zeroRenderRateRequestsNoImage) {
    FakeCielim renderer;
    CielimHarness harness;
    std::string port = renderer.getPort();
    harness.cielim.setPortNumber(port);
    harness.addCamera(2, 0);

    harness.run(3);
    std::vector<Request> const &requests = renderer.stop();

    std::vector<std::string> const expected{"PING", "INIT_SCENE", "SIM_UPDATE", "SIM_UPDATE"};
    EXPECT_EQ(commands(requests), expected);
}

//! When the length frame claims more bytes than the image frame has, the image message is not valid.
TEST(CielimInterfaceLiveStream, rejectsImageLengthLargerThanFrame) {
    FakeCielim renderer;
    renderer.setImage("SHORT");
    renderer.setImageLengthOverride(1'000'000);
    CielimHarness harness;
    std::string port = renderer.getPort();
    harness.cielim.setPortNumber(port);
    harness.addCamera(2, oneSecond);
    ReadFunctor<CameraImageMsgPayload> imageReader = harness.cielim.imageOutMessage.addSubscriber();

    harness.run(2);
    renderer.stop();

    CameraImageMsgPayload const image = imageReader();
    EXPECT_EQ(image.valid, 0);
    EXPECT_EQ(image.imageBufferLength, 0);
    EXPECT_EQ(image.imagePointer, nullptr);
}

//! A body without a spice message has an identity attitude in the saved frames.
TEST(CielimInterfaceFile, unlinkedBodyHasIdentityAttitude) {
    std::filesystem::path const path =
        std::filesystem::path(::testing::TempDir()) / ("cielimFrames_" + std::to_string(::getpid()) + ".bin");
    {
        CielimHarness harness;
        harness.cielim.setOpNavMode(ClosedLoopMode::OPEN_LOOP);
        harness.cielim.setSaveFile(path.string());
        SpiceBody body;
        body.name = "bennu";
        harness.cielim.addCelestialBody(body);
        harness.run(2);
    }

    auto const frames = readFrames(path.string());
    std::filesystem::remove(path);

    ASSERT_EQ(frames.size(), 1u);
    ASSERT_EQ(frames[0].celestialbodies_size(), 1);
    auto const &attitude = frames[0].celestialbodies(0).attitude();
    std::vector<double> const identity{1, 0, 0, 0, 1, 0, 0, 0, 1};
    EXPECT_EQ(std::vector<double>(attitude.begin(), attitude.end()), identity);
}
