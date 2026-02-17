// SPDX-License-Identifier: ISC
// Copyright (c) 2016, Autonomous Vehicle System Lab, University of Colorado at Boulder
// Copyright (c) 2025, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

#ifndef CELESTIAL_BODY_POINT_C_H
#define CELESTIAL_BODY_POINT_C_H

#include <stdint.h>

#include <architecture/_GeneralModuleFiles/sys_model.h>
#include <architecture/messaging/messaging.h>
#include <architecture/msgPayloadDef/AttRefMsgPayload.h>
#include <architecture/msgPayloadDef/EphemerisMsgPayload.h>
#include <architecture/msgPayloadDef/NavTransMsgPayload.h>

#include <architecture/utilities/bskLogging.h>

/*!@brief Data structure for module to compute the two-body celestial pointing navigation solution.
 */
class CelestialTwoBodyPoint_C : public SysModel {
   public:
    void reset(uint64_t callTime) override;
    void updateState(uint64_t callTime) override;
    void parseInputMessages();
    void computeCelestialTwoBodyPoint(uint64_t callTime);
    /* Declare module private variables */
    double singularityThresh;  //!< (r) Threshold for when to fix constraint axis*/
    double R_P1B_N[3];         //!< [m] planet 1 position vector relative to inertial frame, in N-frame components
    double R_P2B_N[3];         //!< [m] planet 2 position vector relative to inertial frame, in N-frame components
    double v_P1B_N[3];         //!< [m/s] planet 1 velocity vector relative to inertial frame, in N-frame components
    double v_P2B_N[3];         //!< [m/s] planet 2 velocity vector relative to inertial frame, in N-frame components
    double a_P1B_N[3];  //!< [m/s^2] planet 1 acceleration vector relative to inertial frame, in N-frame components
    double a_P2B_N[3];  //!< [m/s^2] planet 2 acceleration vector relative to inertial frame, in N-frame components

    /* Declare module IO interfaces */
    Message<AttRefMsgPayload> attRefOutMsg;            //!< The name of the output message*/
    ReadFunctor<EphemerisMsgPayload> celBodyInMsg;     //!< The name of the celestial body message*/
    ReadFunctor<EphemerisMsgPayload> secCelBodyInMsg;  //!< The name of the secondary body to constrain point*/
    ReadFunctor<NavTransMsgPayload> transNavInMsg;     //!< The name of the incoming attitude command*/

    int secCelBodyIsLinked;  //!< flag to indicate if the optional 2nd celestial body message is linked

    /* Output attitude reference data to send */
    AttRefMsgPayload attRefOut;  //!< (-) copy of output reference frame message

    BSKLogger bskLogger = {};  //!< BSK Logging
};

#endif
