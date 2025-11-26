#ifndef ZMQCONNECTOR_H
#define ZMQCONNECTOR_H

#include "cielimMessage.pb.h"
#include "imageDiagnostics.pb.h"
#include <Eigen/Core>
#include <string>
#include <zmq.hpp>

struct ImageData {
    int32_t imageBufferLength;
    void* imageBuffer;
    std::optional<Eigen::Vector2d> centerOfBrightness;
    std::optional<double> coverage;
    std::optional<int32_t> brightPixels;
};

class ZmqConnector {
   public:
    ZmqConnector();
    ~ZmqConnector() = default;

    void connect();
    [[nodiscard]] bool isConnected() const;
    void send(const cielimMessage::CielimMessage& messagePayload);
    ImageData requestImage(size_t cameraId, bool shoudReturnImage = true, bool shouldReturnDiagnostics = true);
    void setComPortNumber(std::string& portNumber);
    void ping();
    void init();

   private:
    std::shared_ptr<zmq::context_t> context;
    std::unique_ptr<zmq::socket_t> requesterSocket;
    std::string comProtocol = "tcp";
    std::string comAddress = "127.0.0.1";
    std::string comPortNumber = "5556";

    static void message_buffer_deallocate(void* data, void* hint);
};

#endif  // ZMQCONNECTOR_H
