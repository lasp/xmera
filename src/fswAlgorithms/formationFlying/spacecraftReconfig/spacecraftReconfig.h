// SPDX-License-Identifier: ISC
// Copyright (c) 2016, Autonomous Vehicle System Lab, University of Colorado at Boulder
// Copyright (c) 2025, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

#ifndef _SPACECRAFT_RECONFIG_H_
#define _SPACECRAFT_RECONFIG_H_

#include <stdint.h>

#include <architecture/_GeneralModuleFiles/sys_model.h>
#include <architecture/messaging/messaging.h>
#include <architecture/msgPayloadDef/AttRefMsgPayload.h>
#include <architecture/msgPayloadDef/NavTransMsgPayload.h>
#include <architecture/msgPayloadDef/ReconfigBurnArrayInfoMsgPayload.h>
#include <architecture/msgPayloadDef/THRArrayConfigMsgPayload.h>
#include <architecture/msgPayloadDef/THRArrayOnTimeCmdMsgPayload.h>
#include <architecture/msgPayloadDef/VehicleConfigMsgPayload.h>

#include <architecture/utilities/bskLogging.h>
#include <architecture/utilities/orbitalMotion.h>

/*! @brief Data structure for the MRP feedback attitude control routine. */
class SpacecraftReconfig : public SysModel {
   public:
    void reset(uint64_t callTime) override;
    void updateState(uint64_t callTime) override;
    void UpdateManeuver(NavTransMsgPayload chiefTransMsgBuffer,
                        NavTransMsgPayload deputyTransMsgBuffer,
                        AttRefMsgPayload attRefInMsgBuffer,
                        THRArrayConfigMsgPayload thrustConfigMsgBuffer,
                        VehicleConfigMsgPayload vehicleConfigMsgBuffer,
                        AttRefMsgPayload* attRefOutMsgBuffer,
                        THRArrayOnTimeCmdMsgPayload* thrustOnMsgBuffer,
                        uint64_t callTime);

    void ScheduleDV(ClassicElements oe_c,
                    ClassicElements oe_d,
                    THRArrayConfigMsgPayload thrustConfigMsgBuffer,
                    VehicleConfigMsgPayload vehicleConfigMsgBuffer);

    /* declare module IO interfaces */
    // in
    ReadFunctor<NavTransMsgPayload> chiefTransInMsg;          //!< chief orbit input msg
    ReadFunctor<NavTransMsgPayload> deputyTransInMsg;         //!< deputy orbit input msg
    ReadFunctor<THRArrayConfigMsgPayload> thrustConfigInMsg;  //!< THR configuration input msg
    ReadFunctor<AttRefMsgPayload> attRefInMsg;                //!< nominal attitude reference input msg
    ReadFunctor<VehicleConfigMsgPayload> vehicleConfigInMsg;  //!< deputy vehicle config msg

    // out
    Message<AttRefMsgPayload> attRefOutMsg;                        //!< attitude reference output msg
    Message<THRArrayOnTimeCmdMsgPayload> onTimeOutMsg;             //!< THR on-time output msg
    Message<ReconfigBurnArrayInfoMsgPayload> burnArrayInfoOutMsg;  //!< array of burn info output msg

    double mu;                   //!< [m^3/s^2] gravity constant of planet being orbited
    double attControlTime;       //!< [s] attitude control margin time (time necessary to change sc's attitude)
    double targetClassicOED[6];  //!< target classic orital element difference, SMA should be normalized
    double resetPeriod;          //!< [s] burn scheduling reset period
    double tCurrent;             //!< [s] timer
    uint64_t prevCallTime;       //!< [ns]
    uint8_t thrustOnFlag;        //!< thrust control
    int attRefInIsLinked;        //!< flag if the attitude reference input message is linked
    ReconfigBurnArrayInfoMsgPayload burnArrayInfoOutMsgBuffer;  //!< msg buffer for burn array info

    BSKLogger bskLogger = {};  //!< BSK Logging
};

#endif
