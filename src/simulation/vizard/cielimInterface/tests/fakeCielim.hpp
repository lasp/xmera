// SPDX-License-Identifier: ISC
// Copyright (c) 2026, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

#ifndef CIELIM_INTERFACE_FAKE_CIELIM_HPP
#define CIELIM_INTERFACE_FAKE_CIELIM_HPP

#include "imageDiagnostics.pb.h"

#include <atomic>
#include <cstdint>
#include <iterator>
#include <optional>
#include <string>
#include <thread>
#include <vector>
#include <zmq.hpp>
#include <zmq_addon.hpp>

namespace cielimtest {
    //! One multipart request that the fake renderer received, one string for each frame.
    using Request = std::vector<std::string>;

    /*! A ZeroMQ reply socket on a free local port that acts as the Cielim renderer. It records each
        request and sends a reply. An image request gets an image frame, a length frame and a
        diagnostics frame. All other requests get a short text reply. */
    class FakeCielim {
    public:
        static constexpr int receiveTimeoutMs = 200;         //!< [ms] poll period of the worker thread
        static constexpr double centerOfBrightnessX = 10.0;  //!< [pixel] center of brightness in the diagnostics
        static constexpr double centerOfBrightnessY = 20.0;  //!< [pixel] center of brightness in the diagnostics

        FakeCielim() : socket(this->context, zmq::socket_type::rep) {
            this->socket.set(zmq::sockopt::linger, 0);
            this->socket.set(zmq::sockopt::rcvtimeo, receiveTimeoutMs);
            this->socket.bind("tcp://127.0.0.1:*");
            std::string const endpoint = this->socket.get(zmq::sockopt::last_endpoint);
            this->port = endpoint.substr(endpoint.rfind(':') + 1);
            this->worker = std::thread([this] { this->run(); });
        }

        ~FakeCielim() {
            this->stop();
        }

        FakeCielim(FakeCielim const &) = delete;
        FakeCielim &operator=(FakeCielim const &) = delete;
        FakeCielim(FakeCielim &&) = delete;
        FakeCielim &operator=(FakeCielim &&) = delete;

        //! Local TCP port of the reply socket.
        [[nodiscard]] std::string const &getPort() const {
            return this->port;
        }

        //! Set the image bytes that the next image requests get.
        void setImage(std::string bytes) {
            this->image = std::move(bytes);
        }

        //! Send this value in the length frame instead of the image size.
        void setImageLengthOverride(int32_t length) {
            this->lengthOverride = length;
        }

        //! Stop the worker thread after the socket is idle and return all received requests.
        std::vector<Request> const &stop() {
            this->stopping = true;
            if (this->worker.joinable()) { this->worker.join(); }
            return this->requests;
        }

    private:
        void run() {
            while (true) {
                std::vector<zmq::message_t> parts;
                auto const received = zmq::recv_multipart(this->socket, std::back_inserter(parts));
                if (!received) {
                    if (this->stopping) { break; }
                    continue;
                }
                Request request;
                for (auto const &part : parts) { request.push_back(part.to_string()); }
                this->requests.push_back(request);
                this->reply(request);
            }
        }

        void reply(Request const &request) {
            if (request.front() == "REQUEST_IMAGE") {
                int32_t const length = this->lengthOverride.value_or(static_cast<int32_t>(this->image.size()));
                imageDiagnostics::DiagnosticData diagnostics;
                diagnostics.set_cob_x(centerOfBrightnessX);
                diagnostics.set_cob_y(centerOfBrightnessY);
                this->socket.send(zmq::buffer(this->image), zmq::send_flags::sndmore);
                this->socket.send(zmq::buffer(&length, sizeof(length)), zmq::send_flags::sndmore);
                this->socket.send(zmq::buffer(diagnostics.SerializeAsString()), zmq::send_flags::none);
                return;
            }
            this->socket.send(zmq::str_buffer("OK"), zmq::send_flags::none);
        }

        std::string port;
        zmq::context_t context;
        zmq::socket_t socket;
        std::thread worker;
        std::atomic<bool> stopping{false};
        std::vector<Request> requests;
        std::string image = "IMAGE";
        std::optional<int32_t> lengthOverride;
    };
}  // namespace cielimtest

#endif  // CIELIM_INTERFACE_FAKE_CIELIM_HPP
