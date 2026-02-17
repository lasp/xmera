// SPDX-License-Identifier: ISC
// Copyright (c) 2023, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

#ifndef LAMBERTSURFACERELATIVEVELOCITY_H
#define LAMBERTSURFACERELATIVEVELOCITY_H

#include <architecture/_GeneralModuleFiles/sys_model.h>
#include <architecture/messaging/messaging.h>
#include <architecture/msgPayloadDef/DesiredVelocityMsgPayload.h>
#include <architecture/msgPayloadDef/EphemerisMsgPayload.h>
#include <architecture/msgPayloadDef/LambertProblemMsgPayload.h>
#include <architecture/utilities/astroConstants.h>
#include <architecture/utilities/bskLogging.h>
#include <architecture/utilities/eigenSupport.h>
#include <array>
#include <vector>

/*! @brief This module computes the inertial velocity corresponding to a given position and relative velocity to the
    celestial body surface
 */
class LambertSurfaceRelativeVelocity : public SysModel {
   public:
    LambertSurfaceRelativeVelocity();
    ~LambertSurfaceRelativeVelocity();

    void reset(uint64_t currentSimNanos) override;
    void updateState(uint64_t currentSimNanos) override;

    ReadFunctor<LambertProblemMsgPayload> lambertProblemInMsg;  //!< lambert problem input message
    ReadFunctor<EphemerisMsgPayload> ephemerisInMsg;            //!< ephemeris input message
    Message<DesiredVelocityMsgPayload> desiredVelocityOutMsg;   //!< desired inertial velocity output message

    BSKLogger bskLogger;  //!< -- BSK Logging

    Eigen::Vector3d vRelativeDesired_S;  //!< [m/s] desired relative velocity, in surface frame S (East-North-Up)
    double time{};                       //!< [s] time for the desired velocity of the spacecraft

   private:
    void readMessages();
    void writeMessages(uint64_t currentSimNanos);

    Eigen::Vector3d r_BN_N;      //!< position of spacecraft, expressed in inertial frame N
    Eigen::Vector3d v_BN_N;      //!< velocity of spacecraft, expressed in inertial frame N
    Eigen::Matrix3d dcm_PN;      //!< DCM of the orbital body fixed frame relative to inertial
    Eigen::Vector3d omega_PN_N;  //!< [r/s] angular velocity of the orbital body relative to inertial
};

#endif
