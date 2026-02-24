// SPDX-License-Identifier: ISC
// Copyright (c) 2016, Autonomous Vehicle System Lab, University of Colorado at Boulder
// Copyright (c) 2025, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

#ifndef EPHEMERIS_CONVERTER_H
#define EPHEMERIS_CONVERTER_H

#include <architecture/_GeneralModuleFiles/sys_model.h>
#include <vector>

#include <architecture/messaging/messaging.h>
#include <architecture/msgPayloadDef/EphemerisMsgPayload.h>
#include <architecture/msgPayloadDef/SpicePlanetStateMsgPayload.h>

#include <architecture/utilities/bskLogging.h>

/*! @brief ephemeric converter class */
class EphemerisConverter : public SysModel {
   public:
    EphemerisConverter();
    ~EphemerisConverter();

    void updateState(uint64_t currentSimNanos);
    void reset(uint64_t currentSimNanos);
    void readInputMessages();  //!< class method
    void convertEphemData(uint64_t clockNow);
    void writeOutputMessages(uint64_t Clock);
    void addSpiceInputMsg(Message<SpicePlanetStateMsgPayload>* msg);

   public:
    std::vector<Message<EphemerisMsgPayload>*> ephemOutMsgs;           //!< vector of planet ephemeris output messages
    std::vector<ReadFunctor<SpicePlanetStateMsgPayload>> spiceInMsgs;  //!< vector of planet spice state input messages

    BSKLogger bskLogger;  //!< -- BSK Logging
   private:
    std::vector<EphemerisMsgPayload> ephemOutBuffers;        //!< output message buffers
    std::vector<SpicePlanetStateMsgPayload> spiceInBuffers;  //!< spice input message copies
};

#endif
