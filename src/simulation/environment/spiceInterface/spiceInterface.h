// SPDX-License-Identifier: ISC
// Copyright (c) 2016, Autonomous Vehicle System Lab, University of Colorado at Boulder
// Copyright (c) 2025, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

#ifndef SpiceInterface_H
#define SpiceInterface_H

#include <architecture/_GeneralModuleFiles/sys_model.h>
#include <architecture/utilities/bskLogging.h>
#include <architecture/utilities/eigenSupport.h>
#include <architecture/utilities/linearAlgebra.h>
#include <map>
#include <vector>

#include <architecture/messaging/messaging.h>
#include <architecture/msgPayloadDef/AttRefMsgPayload.h>
#include <architecture/msgPayloadDef/EpochMsgPayload.h>
#include <architecture/msgPayloadDef/SCStatesMsgPayload.h>
#include <architecture/msgPayloadDef/SpicePlanetStateMsgPayload.h>
#include <architecture/msgPayloadDef/SpiceTimeMsgPayload.h>
#include <architecture/msgPayloadDef/TransRefMsgPayload.h>

/*! @brief spice interface class */
class SpiceInterface : public SysModel {
   public:
    SpiceInterface();
    ~SpiceInterface();

    void updateState(uint64_t currentSimNanos);
    int loadSpiceKernel(char* kernelName, const char* dataPath);
    int unloadSpiceKernel(char* kernelName, const char* dataPath);
    std::string getCurrentTimeString();  //!< class method
    void reset(uint64_t currentSimNanos);
    void initTimeData();
    void computeGPSData();
    void pullSpiceData(std::vector<SpicePlanetStateMsgPayload>* spiceData);
    void writeOutputMessages(uint64_t CurrentClock);
    void clearKeeper();  //!< class method
    void addPlanetNames(std::vector<std::string> planetNames);
    void addSpacecraftNames(std::vector<std::string> spacecraftNames);

   public:
    Message<SpiceTimeMsgPayload> spiceTimeOutMsg;                          //!< spice time sampling output message
    ReadFunctor<EpochMsgPayload> epochInMsg;                               //!< (optional) input epoch message
    std::vector<Message<SpicePlanetStateMsgPayload>*> planetStateOutMsgs;  //!< vector of planet state output messages
    std::vector<Message<SCStatesMsgPayload>*> scStateOutMsgs;  //!< vector of spacecraft state output messages
    std::vector<Message<AttRefMsgPayload>*>
        attRefStateOutMsgs;  //!< vector of spacecraft attitude reference state output messages
    std::vector<Message<TransRefMsgPayload>*>
        transRefStateOutMsgs;  //!< vector of spacecraft translational reference state output messages

    std::string SPICEDataPath;   //!< -- Path on file to SPICE data
    std::string referenceBase;   //!< -- Base reference frame to use
    std::string zeroBase;        //!< -- Base zero point to use for states
    std::string timeOutPicture;  //!< -- Optional parameter used to extract time strings
    bool SPICELoaded;            //!< -- Boolean indicating to reload spice
    int charBufferSize;          //!< -- avert your eyes we're getting SPICE
    uint8_t* spiceBuffer;        //!< -- General buffer to pass down to spice
    std::string UTCCalInit;      //!< -- UTC time string for init time

    std::vector<std::string>
        planetFrames;  //!< -- Optional vector of planet frame names.  Default values are IAU_ + planet name

    bool timeDataInit;         //!< -- Flag indicating whether time has been init
    double J2000ETInit;        //!< s Seconds elapsed since J2000 at init
    double J2000Current;       //!< s Current J2000 elapsed time
    double julianDateCurrent;  //!< s Current JulianDate
    double GPSSeconds;         //!< s Current GPS seconds
    uint16_t GPSWeek;          //!< -- Current GPS week value
    uint64_t GPSRollovers;     //!< -- Count on the number of GPS rollovers

    BSKLogger bskLogger;  //!< -- BSK Logging

   private:
    std::string GPSEpochTime;  //!< -- String for the GPS epoch
    double JDGPSEpoch;         //!< s Epoch for GPS time.  Saved for efficiency

    std::vector<SpicePlanetStateMsgPayload> planetData;
    std::vector<SpicePlanetStateMsgPayload> scData;
};

#endif
