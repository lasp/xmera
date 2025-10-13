/*
 Copyright (c) 2024, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

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

#ifndef CIELIM_INTERFACE_H
#define CIELIM_INTERFACE_H

#include <xmera/sys_model.h>
#include "architecture/messaging/messaging.h"
#include "architecture/msgPayloadDef/CameraImageMsgPayload.h"
#include "architecture/msgPayloadDef/CameraModelMsgPayload.h"
#include "architecture/msgPayloadDef/CameraRenderingMsgPayload.h"
#include "architecture/msgPayloadDef/CelestialBodyParametersMsgPayload.h"
#include "architecture/msgPayloadDef/EpochMsgPayload.h"
#include "architecture/msgPayloadDef/OpNavCOBMsgPayload.h"
#include "architecture/msgPayloadDef/SCStatesMsgPayload.h"
#include "architecture/msgPayloadDef/SpicePlanetStateMsgPayload.h"

#include "architecture/utilities/rigidBodyKinematics.hpp"
#include "simulation/vizard/cielimInterface/zmqConnector.h"
#include "simulation/vizard/cielimInterface/cielimMessage.pb.h"

#include <google/protobuf/io/coded_stream.h>
#include <google/protobuf/util/delimited_message_util.h>
#include <zmq.h>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <vector>

enum class ClosedLoopMode { OPEN_LOOP = 0, ALL_FRAMES = 1, REQUESTED_FRAMES = 2 };

class MessageStatus {
   public:
    uint64_t lastTimeTag = 0xFFFFFFFFFFFFFFFF;  //!< [ns] The previous read time-tag for msg
    bool dataFresh = false;                     //!< [-] Flag indicating that new data has been read
};

/*! Structure defining vizard gravity body values */
class SpiceBody {
   public:
    std::string name{};                                                           //!< [-] celestial body name
    ReadFunctor<SpicePlanetStateMsgPayload> spiceStateMessage{};                  //!< [-] celestial body name
    SpicePlanetStateMsgPayload spiceStatePayload{};                               //!< [-] celestial body name
    ReadFunctor<CelestialBodyParametersMsgPayload> celestialParametersMessage{};  //!< [-] celestial body parameters msg
    CelestialBodyParametersMsgPayload celestialParametersPayload{};               //!< [-] celestial body parameters
    bool isCentralBody = false;                                                   //!< [-] celestial body name
};

/*!  @brief The interface to Cielim via ZMQ and protobuffers
 */
class CielimInterface : public SysModel {
   public:
    void reset(uint64_t currentSimNanos) override;
    void updateState(uint64_t currentSimNanos) override;
    void readBskMessages();
    void writeProtobuffer(uint64_t currentSimNanos);

    void setOpNavMode(ClosedLoopMode mode);
    ClosedLoopMode getOpNavMode() const;
    int64_t getFrameNumber() const;
    void setSaveFile(const std::string &pathAndFilename);
    std::string getSaveFilename() const;
    void closeProtobufFile();
    void setLiveStream(bool liveStreaming);
    void setPortNumber(std::string port);
    void addCelestialBody(const SpiceBody &celestialBodiesList);
    std::vector<SpiceBody> getCelestialBodies() const;

    ReadFunctor<SCStatesMsgPayload> spacecraftMessage;              //!< [-] vector of spacecraft data containers
    ReadFunctor<CameraModelMsgPayload> cameraModelMessage;          //!< [-] incoming camera data message
    ReadFunctor<CameraRenderingMsgPayload> cameraRenderingMessage;  //!< [-] camera rendering message
    ReadFunctor<EpochMsgPayload> epochMessage;                      //!< [-] simulation epoch date/time input msg

    Message<CameraImageMsgPayload> imageOutMessage;            //!< vector of vizard instrument camera output messages
    Message<OpNavCOBMsgPayload> centerOfBrightnessOutMessage;  //!< The true image center of brightness output message

   private:
    void requestImage(uint64_t currentSimNanos);  //!<   request image and store it in output image message
    bool shouldRequestACameraImage(uint64_t currentSimNanos) const;

    bool saveFile{false};    //!< [Bool] Set True if Vizard should save a file of the data.
    bool liveStream{false};  //!< [Bool] Set True if Vizard should receive a live stream of BSK data.
    void *imagePointer;      /*!< [RUN] permanent pointers for the images to be used
                                  without relying on ZMQ because ZMQ will free it (whenever, who knows) */
    ZmqConnector connector{};
    ClosedLoopMode opNavMode{ClosedLoopMode::ALL_FRAMES}; /*!< [int] Set if Unity/Viz couple in direct communication. */
    int64_t frameNumber{-1};  //!< Number of frames that have been updated for TimeStamp message

    SCStatesMsgPayload spacecraftPayload{};   //!< [-] vector of spacecraft data containers
    MessageStatus spacecraftMessageStatus{};  //!< [-] message status of spacecraft data

    EpochMsgPayload epochPayload{};      //!< [-] epoch msg data
    MessageStatus epochMessageStatus{};  //!< [-] ID of the epoch msg

    std::vector<SpicePlanetStateMsgPayload> spiceBodyPayloads;  //!< [-] payloads of planet Spice data
    std::vector<MessageStatus> spiceBodyMessageStatus;          //!< [-] status of the incoming planets' spice data
    std::vector<CelestialBodyParametersMsgPayload> celestialParametersPayloads;  //!< [-] celestial parameters payloads
    std::vector<MessageStatus> celestialParametersMessageStatus;  //!< [-] status of celestial parameter messages
    std::vector<SpiceBody> celestialBodiesList;                   //!< [-] celestial body names

    CameraModelMsgPayload cameraModelPayload{};  //!< [-] camera config buffers
    MessageStatus cameraModelMessageStatus{};    //!< [-] message status of incoming camera data

    CameraRenderingMsgPayload cameraRenderingPayload{};  //!< [-] buffer for camera rendering settings
    MessageStatus cameraRenderingMessageStatus{};        //!< [-] message status of the camera rendering message

    std::string protoFilename{};   //!< Filename for where to save the protobuff message
    std::ofstream outputStream{};  //!< [-] Output file stream opened in reset
};

#endif /* CIELIM_INTERFACE_H */
