// SPDX-License-Identifier: ISC
// Copyright (c) 2016, Autonomous Vehicle System Lab, University of Colorado at Boulder
// Copyright (c) 2025, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

#ifndef _MRP_PD_CONTROL_C_H_
#define _MRP_PD_CONTROL_C_H_

#include <architecture/_GeneralModuleFiles/sys_model.h>
#include <architecture/messaging/messaging.h>
#include <architecture/msgPayloadDef/AttGuidMsgPayload.h>
#include <architecture/msgPayloadDef/CmdTorqueBodyMsgPayload.h>
#include <architecture/msgPayloadDef/VehicleConfigMsgPayload.h>
#include <stdint.h>

/*! @brief Module configuration message definition. */
class MrpPD_C : public SysModel {
   public:
    void reset(uint64_t callTime) override;
    void updateState(uint64_t callTime) override;

    /* declare public module variables */
    double K;                     //!< [rad/sec] Proportional gain applied to MRP errors
    double P;                     //!< [N*m*s]   Rate error feedback gain applied
    double knownTorquePntB_B[3];  //!< [N*m]     known external torque in body frame vector components

    /* declare private module variables */
    double ISCPntB_B[9];  //!< [kg m^2] Spacecraft Inertia

    /* declare module IO interfaces */
    Message<CmdTorqueBodyMsgPayload> cmdTorqueOutMsg;     //!< commanded torque output message
    ReadFunctor<AttGuidMsgPayload> guidInMsg;             //!< attitude guidance input message
    ReadFunctor<VehicleConfigMsgPayload> vehConfigInMsg;  //!< vehicle configuration input message

    BSKLogger bskLogger = {};  //!< BSK Logging
};

#endif
