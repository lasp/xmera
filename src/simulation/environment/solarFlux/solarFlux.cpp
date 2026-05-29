// SPDX-License-Identifier: ISC
// Copyright (c) 2016, Autonomous Vehicle System Lab, University of Colorado at Boulder
// Copyright (c) 2025, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

#include "solarFlux.h"
#include <architecture/utilities/astroConstants.h>

/*! This method is used to reset the module. Currently no tasks are required.
 @return void
 */
void SolarFlux::reset(uint64_t currentSimNanos) {
    // check if input message has not been included
    if (!this->sunPositionInMsg.isLinked()) {
        bskLogger.bskLog(BSK_ERROR, "solarFlux.sunPositionInMsg was not linked.");
    }
    if (!this->spacecraftStateInMsg.isLinked()) {
        bskLogger.bskLog(BSK_ERROR, "solarFlux.spacecraftStateInMsg was not linked.");
    }

    return;
}

/*! Read Messages and scale the solar flux then write it out
 @return void
 */
void SolarFlux::updateState(uint64_t currentSimNanos) {
    this->readMessages();

    /*! - evaluate spacecraft position relative to the sun in N frame components */
    auto r_SSc_N = this->r_SN_N - this->r_ScN_N;

    /*! - compute the scalar distance to the sun.  The following math requires this to be in km. */
    double dist_SSc_N = r_SSc_N.norm() / 1000;  // to km

    /*! - compute the local solar flux value */
    this->fluxAtSpacecraft = SOLAR_FLUX_EARTH * pow(AU, 2) / pow(dist_SSc_N, 2) * this->eclipseFactor;

    this->writeMessages(currentSimNanos);
}

/*! This method is used to  read messages and save values to member attributes
 @return void
 */
void SolarFlux::readMessages() {
    /*! - read in planet state message (required) */
    SpicePlanetStateMsgPayload sunPositionMsgData;
    sunPositionMsgData = this->sunPositionInMsg();
    this->r_SN_N = Eigen::Vector3d(sunPositionMsgData.PositionVector);

    /*! - read in spacecraft state message (required) */
    SCStatesMsgPayload scStatesMsgData;
    scStatesMsgData = this->spacecraftStateInMsg();
    this->r_ScN_N = Eigen::Vector3d(scStatesMsgData.r_BN_N);

    /*! - read in eclipse message (optional) */
    if (this->eclipseInMsg.isLinked()) {
        EclipseMsgPayload eclipseInMsgData;
        eclipseInMsgData = this->eclipseInMsg();
        this->eclipseFactor = eclipseInMsgData.shadowFactor;
    }
}

/*! This method is used to write the output flux message
 @return void
 */
void SolarFlux::writeMessages(uint64_t currentSimNanos) {
    SolarFluxMsgPayload fluxMsgOutData = {this->fluxAtSpacecraft};
    this->solarFluxOutMsg.write(fluxMsgOutData, this->moduleID, currentSimNanos);
}
