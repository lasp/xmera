#include "zmqConnector.h"

ZmqConnector::ZmqConnector() = default;

void ZmqConnector::connect() {
    if (!this->isConnected()) {
        this->context = std::make_shared<zmq::context_t>();
        this->requesterSocket = std::make_unique<zmq::socket_t>(*this->context, ZMQ_REQ);
        std::cout << this->comAddress << ":" << this->comPortNumber << std::endl;
        this->requesterSocket->connect(this->comProtocol + "://" + this->comAddress + ":" + this->comPortNumber);
    }
    this->ping();
    this->init();
}

bool ZmqConnector::isConnected() const {
    if (this->requesterSocket) {
        return (this->requesterSocket->handle() != nullptr);
    }
    return false;
}

void ZmqConnector::send(const cielimMessage::CielimMessage &message) {
    /*! - send protobuffer raw over zmq_socket */
    size_t byteCount = message.ByteSizeLong();
    void *serialized_message = malloc(byteCount);
    message.SerializeToArray(serialized_message, (int)byteCount);
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

void ZmqConnector::message_buffer_deallocate(void *data, void *hint) { free(data); }

ImageData ZmqConnector::requestImage(size_t cameraId, bool shouldReturnImage) {
    this->requesterSocket->send(zmq::message_t("REQUEST_IMAGE", 13), zmq::send_flags::sndmore);
    this->requesterSocket->send(zmq::message_t(std::to_string(cameraId)), zmq::send_flags::sndmore);
    this->requesterSocket->send(zmq::message_t(std::to_string(shouldReturnImage).c_str(), sizeof(char)),
                                zmq::send_flags::none);

    // SAFETY: it's okay to discard these [[nodiscard]] values because
    //   1) the returned optional could only be empty if ZeroMQ fails due to EAGAIN on a non-blocking socket;
    //      but our socket is not non-blocking
    //   2) the returned length in the (present) optional is recoverable from the given message's `.size()` method.
    auto imageLengthMessage = zmq::message_t();
    auto imageMessage = zmq::message_t();
    auto centerOfBrightnessX = zmq::message_t();
    auto centerOfBrightnessY = zmq::message_t();
    static_cast<void>(this->requesterSocket->recv(imageMessage, zmq::recv_flags::none));
    static_cast<void>(this->requesterSocket->recv(imageLengthMessage, zmq::recv_flags::none));
    auto cobXMsgSize = this->requesterSocket->recv(centerOfBrightnessX, zmq::recv_flags::none);
    auto cobYMsgSize = this->requesterSocket->recv(centerOfBrightnessY, zmq::recv_flags::none);

    const int32_t *lengthPoint = imageLengthMessage.data<int32_t>();
    const void *imagePoint = imageMessage.data();
    int32_t imageBufferLength = *lengthPoint;
    void *image = malloc(imageBufferLength * sizeof(char));
    memcpy(image, imagePoint, imageBufferLength * sizeof(char));

    auto returnData = ImageData();
    returnData.imageBuffer = image;

    returnData.imageBufferLength = imageBufferLength;
    returnData.centerOfBrightness = std::nullopt;

    if (cobXMsgSize.has_value() && cobYMsgSize.has_value()) {
        returnData.centerOfBrightness =
            Eigen::Vector2d(*centerOfBrightnessX.data<double>(), *centerOfBrightnessY.data<double>());
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

void ZmqConnector::setComPortNumber(std::string &portNumber) { this->comPortNumber = portNumber; }
