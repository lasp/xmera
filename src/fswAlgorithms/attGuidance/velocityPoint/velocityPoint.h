// SPDX-License-Identifier: ISC
// Copyright (c) 2016, Autonomous Vehicle System Lab, University of Colorado at Boulder
// Copyright (c) 2025, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

#ifndef _VELOCITY_POINT_
#define _VELOCITY_POINT_

#include <stdint.h>

#include <architecture/_GeneralModuleFiles/sys_model.h>
#include <architecture/messaging/messaging.h>
#include <architecture/msgPayloadDef/AttRefMsgPayload.h>
#include <architecture/msgPayloadDef/EphemerisMsgPayload.h>
#include <architecture/msgPayloadDef/NavTransMsgPayload.h>
#include <architecture/utilities/bskLogging.h>

/*!@brief Data structure for module to compute the orbital velocity spinning pointing navigation solution.
 */
class VelocityPoint : public SysModel {
   public:
    void reset(uint64_t callTime) override;
    void updateState(uint64_t callTime) override;
    void computeVelocityPointingReference(double r_BN_N[3],
                                          double v_BN_N[3],
                                          double celBdyPositonVector[3],
                                          double celBdyVelocityVector[3],
                                          AttRefMsgPayload* attRefOut);

    /* declare module private variables */
    double mu;  //!< Planet gravitational parameter

    /* declare module IO interfaces */
    Message<AttRefMsgPayload> attRefOutMsg;         //!<        The name of the output message
    ReadFunctor<NavTransMsgPayload> transNavInMsg;  //!<        The name of the incoming attitude command
    ReadFunctor<EphemerisMsgPayload> celBodyInMsg;  //!<        The name of the celestial body message

    int planetMsgIsLinked;  //!<        flag if the planet message is linked

    BSKLogger bskLogger{};  //!< BSK Logging
};

#endif
