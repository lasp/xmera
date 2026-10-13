// SPDX-License-Identifier: ISC
// Copyright (c) 2026, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

#ifndef VIZ_INTERFACE_FAKE_VIZARD_HPP
#define VIZ_INTERFACE_FAKE_VIZARD_HPP

#include <zmq.hpp>
#include <zmq_addon.hpp>

#include <atomic>
#include <cstdint>
#include <iterator>
#include <optional>
#include <string>
#include <thread>
#include <vector>

namespace viztest {

    //! One multipart request that the fake Vizard received, one string for each frame.
    using Request = std::vector<std::string>;

    /*! A ZeroMQ reply socket on a free local port that acts as Vizard. It records each request
        and sends a reply. An image request gets a length frame and an image frame. All other
        requests get a short text reply. */
    class FakeVizard {
       public:
        static constexpr int receiveTimeoutMs = 200;  //!< [ms] poll period of the worker thread

        FakeVizard() : socket(this->context, zmq::socket_type::rep) {
            this->socket.set(zmq::sockopt::linger, 0);
            this->socket.set(zmq::sockopt::rcvtimeo, receiveTimeoutMs);
            this->socket.bind("tcp://127.0.0.1:*");
            std::string const endpoint = this->socket.get(zmq::sockopt::last_endpoint);
            this->port = endpoint.substr(endpoint.rfind(':') + 1);
            this->worker = std::thread([this] { this->run(); });
        }

        ~FakeVizard() { this->stop(); }

        FakeVizard(FakeVizard const&) = delete;
        FakeVizard& operator=(FakeVizard const&) = delete;
        FakeVizard(FakeVizard&&) = delete;
        FakeVizard& operator=(FakeVizard&&) = delete;

        //! Local TCP port of the reply socket.
        [[nodiscard]] std::string const& getPort() const { return this->port; }

        //! Set the image bytes that the next image requests get.
        void setImage(std::string bytes) { this->image = std::move(bytes); }

        //! Send this value in the length frame instead of the image size.
        void setImageLengthOverride(int32_t length) { this->lengthOverride = length; }

        //! Stop the worker thread after the socket is idle and return all received requests.
        std::vector<Request> const& stop() {
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
                for (auto const& part : parts) { request.push_back(part.to_string()); }
                this->requests.push_back(request);
                this->reply(request);
            }
        }

        void reply(Request const& request) {
            if (request.front().starts_with("REQUEST_IMAGE_")) {
                int32_t const length = this->lengthOverride.value_or(static_cast<int32_t>(this->image.size()));
                uint32_t const bigEndian = __builtin_bswap32(static_cast<uint32_t>(length));
                this->socket.send(zmq::buffer(&bigEndian, sizeof(bigEndian)), zmq::send_flags::sndmore);
                this->socket.send(zmq::buffer(this->image), zmq::send_flags::none);
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

}  // namespace viztest

#endif  // VIZ_INTERFACE_FAKE_VIZARD_HPP
