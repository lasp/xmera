// SPDX-License-Identifier: ISC
// Copyright (c) 2016, Autonomous Vehicle System Lab, University of Colorado at Boulder
// Copyright (c) 2025, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

#ifndef planetEphemeris_H
#define planetEphemeris_H

#include <architecture/_GeneralModuleFiles/sys_model.h>
#include <vector>

#include <architecture/messaging/messaging.h>
#include <architecture/msgPayloadDef/SpicePlanetStateMsgPayload.h>

#include <architecture/utilities/bskLogging.h>
#include <architecture/utilities/orbitalMotion.h>

/*! @brief planet ephemeris class */
class PlanetEphemeris : public SysModel {
   public:
    PlanetEphemeris();
    ~PlanetEphemeris();

    void reset(uint64_t currentSimNanos);
    void updateState(uint64_t currentSimNanos);

    void setPlanetNames(std::vector<std::string> planetNames);

   public:
    std::vector<Message<SpicePlanetStateMsgPayload>*> planetOutMsgs;  //!< -- vector of planet state output messages

    std::vector<ClassicElements> planetElements;  //!< -- Vector of planet classical orbit elements

    std::vector<double> rightAscension;  //!< [r] right ascension of the north pole rotation axis (pos. 3-axis)
    std::vector<double> declination;     //!< [r] Declination of the north pole rotation axis (neg. 2-axis)
    std::vector<double> lst0;            //!< [r] initial planet local sidereal time angle (pos. 3-axis)

    std::vector<double> rotRate;  //!< [r/s] planet rotation rate

    BSKLogger bskLogger;  //!< -- BSK Logging

   private:
    std::vector<std::string> planetNames;  //!< -- Vector of planet names
    double epochTime;                      //!< [s] time of provided planet ephemeris epoch
    int computeAttitudeFlag;               //!< -- flag indicating if the planet orienation information is provided
};

#endif
