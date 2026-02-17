// SPDX-License-Identifier: ISC
// Copyright (c) 2016, Autonomous Vehicle System Lab, University of Colorado at Boulder
// Copyright (c) 2025, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

#pragma once

#include <architecture/_GeneralModuleFiles/sys_model.h>
#include <architecture/utilities/bskLogging.h>
#include <Eigen/Dense>

#include <architecture/messaging/messaging.h>
#include <architecture/msgPayloadDef/EclipseMsgPayload.h>
#include <architecture/msgPayloadDef/SCStatesMsgPayload.h>
#include <architecture/msgPayloadDef/SolarFluxMsgPayload.h>
#include <architecture/msgPayloadDef/SpicePlanetStateMsgPayload.h>

/*! @brief solar flux class */
class SolarFlux : public SysModel {
   public:
    SolarFlux() {};
    ~SolarFlux() {};

    void reset(uint64_t currentSimNanos) override;
    void updateState(uint64_t currentSimNanos) override;
    void writeMessages(uint64_t currentSimNanos);
    void readMessages();

   public:
    ReadFunctor<SpicePlanetStateMsgPayload> sunPositionInMsg;  //!< sun state input message
    ReadFunctor<SCStatesMsgPayload> spacecraftStateInMsg;      //!< spacecraft state input message
    Message<SolarFluxMsgPayload> solarFluxOutMsg;              //!< solar flux output message
    ReadFunctor<EclipseMsgPayload> eclipseInMsg;               //!< (optional) eclipse input message

    BSKLogger bskLogger;  //!< -- BSK Logging

   private:
    double fluxAtSpacecraft;     //!< [W/m2]
    double eclipseFactor = 1.0;  //!< [] 1.0 is full sun, 0.0 is full eclipse
    Eigen::Vector3d r_SN_N;      //!< [m] sun position
    Eigen::Vector3d r_ScN_N;     //!< [m] s/c position
};
