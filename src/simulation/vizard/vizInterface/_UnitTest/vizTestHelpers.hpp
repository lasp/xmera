// SPDX-License-Identifier: ISC
// Copyright (c) 2026, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

#ifndef VIZ_INTERFACE_TEST_HELPERS_HPP
#define VIZ_INTERFACE_TEST_HELPERS_HPP

#include "fakeVizard.hpp"
#include "vizInterface.h"

#include <gtest/gtest.h>

#include <google/protobuf/io/zero_copy_stream_impl.h>
#include <google/protobuf/util/delimited_message_util.h>

#include <cstdint>
#include <deque>
#include <filesystem>
#include <fstream>
#include <string>
#include <vector>

#include <unistd.h>

namespace viztest {

    constexpr uint64_t oneSecond = 1'000'000'000;

    //! Device counts of one test spacecraft.
    struct SpacecraftLayout {
        std::string name;
        size_t numRW = 0;
        size_t numThr = 0;
        size_t numCss = 0;
    };

    /*! Owns a VizInterface and the input messages that it reads. The messages are in deques,
        thus their addresses do not change when more messages are added. */
    class VizHarness {
       public:
        //! Add a spacecraft with linked state, RW, thruster and CSS messages.
        void addSpacecraft(SpacecraftLayout const& layout) {
            VizSpacecraftData data;
            data.spacecraftName = layout.name;
            data.scStateInMsg.subscribeTo(&this->scStateMsgs.emplace_back());
            for (size_t i = 0; i < layout.numRW; i++) { data.rwInMsgs.push_back(this->rwMsgs.emplace_back().addSubscriber()); }
            for (size_t i = 0; i < layout.numThr; i++) {
                data.thrInMsgs.push_back(this->thrMsgs.emplace_back().addSubscriber());
                ThrClusterMap info;
                info.thrTag = layout.name + "_thr";
                data.thrInfo.push_back(info);
            }
            for (size_t i = 0; i < layout.numCss; i++) {
                data.cssInMsgs.push_back(this->cssMsgs.emplace_back().addSubscriber());
            }
            this->viz.scData.push_back(data);
        }

        //! Add a celestial body. A linked body reads a spice message, an unlinked body keeps default states.
        void addPlanet(std::string const& name, bool linked) {
            GravBodyInfo info;
            info.bodyName = name;
            info.mu = 3.986e14;
            info.radEquator = 6.378e6;
            info.radiusRatio = 1.0;
            this->viz.gravBodyInformation.push_back(info);
            ReadFunctor<SpicePlanetStateMsgPayload> reader;
            if (linked) { reader = this->spiceMsgs.emplace_back().addSubscriber(); }
            this->viz.spiceInMsgs.push_back(reader);
        }

        //! Add an instrument camera that takes one image each renderRate nanoseconds.
        void addCamera(int64_t cameraID, uint64_t renderRate) {
            CameraConfigMsgPayload config = {};
            config.cameraID = cameraID;
            config.isOn = 1;
            config.renderRate = renderRate;
            config.fieldOfView = 0.5;
            config.resolution[0] = 512;
            config.resolution[1] = 512;
            Message<CameraConfigMsgPayload>& msg = this->cameraMsgs.emplace_back();
            msg.write(config, 0, 0);
            this->viz.addCamMsgToModule(&msg);
        }

        //! Write every input message at time t, then update the module.
        void step(uint64_t t) {
            double const seconds = static_cast<double>(t) / static_cast<double>(oneSecond);
            for (size_t i = 0; i < this->scStateMsgs.size(); i++) {
                SCStatesMsgPayload payload = {};
                payload.r_BN_N[0] = 7.0e6 + static_cast<double>(i);
                payload.v_BN_N[1] = 7.5e3;
                payload.sigma_BN[2] = 0.1 * seconds;
                this->scStateMsgs[i].write(payload, 0, t);
            }
            for (auto& msg : this->rwMsgs) {
                RWConfigLogMsgPayload payload = {};
                payload.Omega = 100.0 * seconds;
                payload.Omega_max = 600.0;
                payload.gsHat_B[0] = 1.0;
                msg.write(payload, 0, t);
            }
            for (auto& msg : this->thrMsgs) {
                THROutputMsgPayload payload = {};
                payload.maxThrust = 1.0;
                payload.thrustForce = 0.5;
                payload.thrusterDirection[2] = 1.0;
                msg.write(payload, 0, t);
            }
            for (auto& msg : this->cssMsgs) {
                CSSConfigLogMsgPayload payload = {};
                payload.signal = 0.25;
                payload.maxSignal = 1.0;
                payload.nHat_B[0] = 1.0;
                msg.write(payload, 0, t);
            }
            for (auto& msg : this->spiceMsgs) {
                SpicePlanetStateMsgPayload payload = {};
                payload.PositionVector[0] = 1.5e11;
                payload.J20002Pfix[0][0] = payload.J20002Pfix[1][1] = payload.J20002Pfix[2][2] = 1.0;
                msg.write(payload, 0, t);
            }
            this->viz.updateState(t);
        }

        //! Reset the module, then run steps at 0, 1, ..., (numSteps - 1) seconds.
        void run(size_t numSteps) {
            this->viz.reset(0);
            for (size_t k = 0; k < numSteps; k++) { this->step(k * oneSecond); }
        }

        VizInterface viz;

       private:
        std::deque<Message<SCStatesMsgPayload>> scStateMsgs;
        std::deque<Message<RWConfigLogMsgPayload>> rwMsgs;
        std::deque<Message<THROutputMsgPayload>> thrMsgs;
        std::deque<Message<CSSConfigLogMsgPayload>> cssMsgs;
        std::deque<Message<SpicePlanetStateMsgPayload>> spiceMsgs;
        std::deque<Message<CameraConfigMsgPayload>> cameraMsgs;
    };

    //! Return the first frame of each request, in order.
    inline std::vector<std::string> commands(std::vector<Request> const& requests) {
        std::vector<std::string> names;
        for (auto const& request : requests) { names.push_back(request.front()); }
        return names;
    }

    //! Parse the payload frame of a SIM_UPDATE request.
    inline vizProtobufferMessage::VizMessage parseFrame(std::string const& bytes) {
        vizProtobufferMessage::VizMessage frame;
        frame.ParseFromString(bytes);
        return frame;
    }

    //! A file path in the gtest temporary directory. The destructor removes the file.
    class TempFile {
       public:
        explicit TempFile(std::string const& name)
            : path(std::filesystem::path(::testing::TempDir()) / (name + "_" + std::to_string(::getpid()) + ".bin")) {}
        ~TempFile() { std::filesystem::remove(this->path); }
        TempFile(TempFile const&) = delete;
        TempFile& operator=(TempFile const&) = delete;
        TempFile(TempFile&&) = delete;
        TempFile& operator=(TempFile&&) = delete;

        [[nodiscard]] std::string string() const { return this->path.string(); }

       private:
        std::filesystem::path path;
    };

    //! Read all length-delimited frames of a saved Vizard file.
    inline std::vector<vizProtobufferMessage::VizMessage> readFrames(std::string const& path) {
        std::ifstream input(path, std::ios::binary);
        google::protobuf::io::IstreamInputStream stream(&input);
        std::vector<vizProtobufferMessage::VizMessage> frames;
        while (true) {
            vizProtobufferMessage::VizMessage frame;
            bool cleanEof = false;
            if (!google::protobuf::util::ParseDelimitedFromZeroCopyStream(&frame, &stream, &cleanEof)) { break; }
            frames.push_back(frame);
        }
        return frames;
    }

}  // namespace viztest

#endif  // VIZ_INTERFACE_TEST_HELPERS_HPP
