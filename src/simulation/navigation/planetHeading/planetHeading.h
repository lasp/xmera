// SPDX-License-Identifier: ISC
// Copyright (c) 2016, Autonomous Vehicle System Lab, University of Colorado at Boulder
// Copyright (c) 2025, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

#pragma once

#include <architecture/_GeneralModuleFiles/sys_model.h>
#include <architecture/utilities/bskLogging.h>
#include <architecture/utilities/eigenMRP.h>
#include <Eigen/Dense>

#include <architecture/messaging/messaging.h>
#include <architecture/msgPayloadDef/BodyHeadingMsgPayload.h>
#include <architecture/msgPayloadDef/SCStatesMsgPayload.h>
#include <architecture/msgPayloadDef/SpicePlanetStateMsgPayload.h>

/*! @brief planet heading class */
class PlanetHeading : public SysModel {
   public:
    PlanetHeading();
    ~PlanetHeading() {};

    void updateState(uint64_t currentSimNanos) override;
    void reset(uint64_t currentSimNanos) override;
    void writeMessages(uint64_t currentSimNanos);
    void readMessages();

   public:
    ReadFunctor<SpicePlanetStateMsgPayload> planetPositionInMsg;  //!< planet state input message
    ReadFunctor<SCStatesMsgPayload> spacecraftStateInMsg;         //!< spacecraft state input message
    Message<BodyHeadingMsgPayload> planetHeadingOutMsg;           //!< body heading output message

    BSKLogger bskLogger;  //!< -- BSK Logging

   private:
    Eigen::Vector3d r_PN_N;     //!< [m] planet position
    Eigen::Vector3d r_BN_N;     //!< [m] s/c position
    Eigen::Vector3d rHat_PB_B;  //!< [] planet heading in s/c body frame (unit mag)
    Eigen::MRPd sigma_BN;       //!< [] s/c body att wrt inertial
};
