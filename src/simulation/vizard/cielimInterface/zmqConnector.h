/*
 ISC License

 Copyright (c) 2023, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

 Permission to use, copy, modify, and/or distribute this software for any
 purpose with or without fee is hereby granted, provided that the above
 copyright notice and this permission notice appear in all copies.

 THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
 WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
 MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
 ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
 WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
 ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
 OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.

 */

#ifndef ZMQCONNECTOR_H
#define ZMQCONNECTOR_H

#include "simulation/vizard/cielimInterface/cielimMessage.pb.h"
#include <Eigen/Core>
#include <string>
#include <zmq.hpp>

struct ImageData {
    int32_t imageBufferLength;
    void *imageBuffer;
    std::optional<Eigen::Vector2d> centerOfBrightness;
};

class ZmqConnector {
   public:
    ZmqConnector();
    ~ZmqConnector() = default;

    void connect();
    [[nodiscard]] bool isConnected() const;
    void send(const cielimMessage::CielimMessage &messagePayload);
    ImageData requestImage(size_t cameraId, bool shoudReturnImage = true);
    void setComPortNumber(std::string &portNumber);
    void ping();
    void init();

   private:
    std::shared_ptr<zmq::context_t> context;
    std::unique_ptr<zmq::socket_t> requesterSocket;
    std::string comProtocol = "tcp";
    std::string comAddress = "127.0.0.1";
    std::string comPortNumber = "5556";

    static void message_buffer_deallocate(void *data, void *hint);
};

#endif  // ZMQCONNECTOR_H
