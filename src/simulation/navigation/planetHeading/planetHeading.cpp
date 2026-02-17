// SPDX-License-Identifier: ISC
// Copyright (c) 2016, Autonomous Vehicle System Lab, University of Colorado at Boulder
// Copyright (c) 2025, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

#include "planetHeading.h"
#include <architecture/utilities/astroConstants.h>
#include <architecture/utilities/eigenSupport.h>

/*! Customer constructor just sets the spacecraftSTateInMsg by default*/
PlanetHeading::PlanetHeading() {}

/*! This method reads messages, calculates the planet heading, and writes out the heading message
 @return void
 */
void PlanetHeading::updateState(uint64_t currentSimNanos) {
    this->readMessages();

    /*! - evaluate planet position relative to the s/c body in body frame components */
    auto r_PB_N = this->r_PN_N - this->r_BN_N;
    /*! - normalize and convert to body frame */
    this->rHat_PB_B = (this->sigma_BN.toRotationMatrix().transpose() * r_PB_N).normalized();

    this->writeMessages(currentSimNanos);
}

/*! Read input messages and save data to member variables
 @return void
 */
void PlanetHeading::readMessages() {
    SpicePlanetStateMsgPayload planetPositionMsgData;
    /*! - read in planet state message (required) */
    planetPositionMsgData = this->planetPositionInMsg();
    this->r_PN_N = Eigen::Vector3d(planetPositionMsgData.PositionVector);

    SCStatesMsgPayload scStatesMsgData;
    /*! - read in spacecraft state message (required) */
    scStatesMsgData = this->spacecraftStateInMsg();
    this->r_BN_N = Eigen::Vector3d(scStatesMsgData.r_BN_N);
    this->sigma_BN = Eigen::MRPd(scStatesMsgData.sigma_BN);
}

/*! This method is used to write out the planet heading message
 @return void
 */
void PlanetHeading::writeMessages(uint64_t currentSimNanos) {
    BodyHeadingMsgPayload planetHeadingOutMsgData;
    planetHeadingOutMsgData = BodyHeadingMsgPayload{};
    eigenVectorToCArray(this->rHat_PB_B, planetHeadingOutMsgData.rHat_XB_B);

    /*! - write the output message */
    this->planetHeadingOutMsg.write(&planetHeadingOutMsgData, this->moduleID, currentSimNanos);
}

/*! This method is used to reset the module. Currently no tasks are required.
 @return void
 */
void PlanetHeading::reset(uint64_t currentSimNanos) {
    // check if input message has not been included
    if (!this->planetPositionInMsg.isLinked()) {
        bskLogger.bskLog(BSK_ERROR, "planetHeading.planetPositionInMsg was not linked.");
    }
    if (!this->spacecraftStateInMsg.isLinked()) {
        bskLogger.bskLog(BSK_ERROR, "planetHeading.spacecraftStateInMsg was not linked.");
    }

    return;
}
