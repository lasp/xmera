// SPDX-License-Identifier: ISC
// Copyright (c) 2026, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

#include "cielimInterface.h"
#include "fakeCielim.hpp"

#include <gtest/gtest.h>

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
