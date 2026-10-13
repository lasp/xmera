// SPDX-License-Identifier: ISC
// Copyright (c) 2023, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

#include "zmqConnector.h"

ZmqConnector::ZmqConnector() = default;

void ZmqConnector::connect() {
    if (!this->isConnected()) {
        this->context = std::make_shared<zmq::context_t>();
        this->requesterSocket = std::make_unique<zmq::socket_t>(*this->context, ZMQ_REQ);
        // Do not wait for unsent messages when the socket closes. Thus the destructor does not
        // block when the renderer is not available.
        this->requesterSocket->set(zmq::sockopt::linger, 0);
        std::cout << this->comAddress << ":" << this->comPortNumber << std::endl;
        this->requesterSocket->connect(this->comProtocol + "://" + this->comAddress + ":" + this->comPortNumber);
    }
    this->ping();
    this->init();
}

bool ZmqConnector::isConnected() const {
    if (this->requesterSocket) { return (this->requesterSocket->handle() != nullptr); }
    return false;
}

void ZmqConnector::send(cielimMessage::CielimMessage const &message) {
    /*! - send protobuffer raw over zmq_socket */
    size_t byteCount = message.ByteSizeLong();
    void* serialized_message = malloc(byteCount);
    message.SerializeToArray(serialized_message, (int) byteCount);
    auto payload = zmq::message_t(serialized_message, byteCount, ZmqConnector::message_buffer_deallocate, nullptr);

    this->requesterSocket->send(zmq::message_t("SIM_UPDATE", 10), zmq::send_flags::sndmore);
    this->requesterSocket->send(payload, zmq::send_flags::none);

    auto pong = zmq::message_t();
    // SAFETY: it's okay to discard this [[nodiscard]] value because
    //   1) the returned optional could only be empty if ZeroMQ fails due to EAGAIN on a non-blocking socket;
    //      but our socket is not non-blocking
    //   2) the returned length in the (present) optional is recoverable from `pong.size()`.
    static_cast<void>(this->requesterSocket->recv(pong, zmq::recv_flags::none));
}

void ZmqConnector::message_buffer_deallocate(void* data, void* hint) {
    free(data);
}

ImageData ZmqConnector::requestImage(size_t cameraId, bool shouldReturnImage, bool shouldReturnDiagnostics) {
    this->requesterSocket->send(zmq::message_t("REQUEST_IMAGE", 13), zmq::send_flags::sndmore);
    this->requesterSocket->send(zmq::message_t(std::to_string(cameraId)), zmq::send_flags::sndmore);
    this->requesterSocket->send(
        zmq::message_t(std::to_string(shouldReturnImage).c_str(), sizeof(char)),
        zmq::send_flags::sndmore
    );
    this->requesterSocket->send(
        zmq::message_t(std::to_string(shouldReturnDiagnostics).c_str(), sizeof(char)),
        zmq::send_flags::none
    );

    // SAFETY: it's okay to discard these [[nodiscard]] values because
    //   1) the returned optional could only be empty if ZeroMQ fails due to EAGAIN on a non-blocking socket;
    //      but our socket is not non-blocking
    //   2) the returned length in the (present) optional is recoverable from the given message's `.size()` method.
    auto imageLengthMessage = zmq::message_t();
    auto imageMessage = zmq::message_t();
    auto diagnosticsMessage = zmq::message_t();
    static_cast<void>(this->requesterSocket->recv(imageMessage, zmq::recv_flags::none));
    static_cast<void>(this->requesterSocket->recv(imageLengthMessage, zmq::recv_flags::none));
    static_cast<void>(this->requesterSocket->recv(diagnosticsMessage, zmq::recv_flags::none));

    // The length must be in the length frame and must not be larger than the image frame.
    // Otherwise the image is empty, and the caller writes an image message that is not valid.
    int32_t imageBufferLength = 0;
    if (imageLengthMessage.size() >= sizeof(int32_t)) {
        memcpy(&imageBufferLength, imageLengthMessage.data(), sizeof(int32_t));
    }
    if (imageBufferLength < 0 || static_cast<size_t>(imageBufferLength) > imageMessage.size()) {
        imageBufferLength = 0;
    }
    void* image = nullptr;
    if (imageBufferLength > 0) {
        image = malloc(imageBufferLength * sizeof(char));
        memcpy(image, imageMessage.data(), imageBufferLength * sizeof(char));
    }

    auto returnData = ImageData();
    returnData.imageBuffer = image;

    returnData.imageBufferLength = imageBufferLength;
    returnData.centerOfBrightness = std::nullopt;
    imageDiagnostics::DiagnosticData diagnostics;
    if (diagnostics.ParseFromArray(diagnosticsMessage.data(), static_cast<int>(diagnosticsMessage.size()))) {
        returnData.centerOfBrightness = Eigen::Vector2d(diagnostics.cob_x(), diagnostics.cob_y());
        returnData.coverage = diagnostics.coverage();
        returnData.brightPixels = diagnostics.totalbrightpixels();
    }

    return returnData;
}

void ZmqConnector::ping() {
    this->requesterSocket->send(zmq::message_t("PING", 4), zmq::send_flags::none);
    auto message = zmq::message_t();
    // SAFETY: it's okay to discard this [[nodiscard]] value because
    //   1) the returned optional could only be empty if ZeroMQ fails due to EAGAIN on a non-blocking socket;
    //      but our socket is not non-blocking
    //   2) the returned length in the (present) optional is recoverable from `pong.size()`.
    static_cast<void>(this->requesterSocket->recv(message, zmq::recv_flags::none));
}

void ZmqConnector::init() {
    this->requesterSocket->send(zmq::message_t("INIT_SCENE", 10), zmq::send_flags::none);
    auto message = zmq::message_t();
    // SAFETY: it's okay to discard this [[nodiscard]] value because
    //   1) the returned optional could only be empty if ZeroMQ fails due to EAGAIN on a non-blocking socket;
    //      but our socket is not non-blocking
    //   2) the returned length in the (present) optional is recoverable from `pong.size()`.
    static_cast<void>(this->requesterSocket->recv(message, zmq::recv_flags::none));
}

void ZmqConnector::setComPortNumber(std::string &portNumber) {
    this->comPortNumber = portNumber;
}
