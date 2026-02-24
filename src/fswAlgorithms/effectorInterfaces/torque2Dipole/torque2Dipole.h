// SPDX-License-Identifier: ISC
// Copyright (c) 2021, Autonomous Vehicle System Lab, University of Colorado at Boulder
// Copyright (c) 2025, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

#ifndef TORQUE2DIPOLE_H
#define TORQUE2DIPOLE_H

#include <architecture/_GeneralModuleFiles/sys_model.h>
#include <architecture/messaging/messaging.h>
#include <architecture/msgPayloadDef/CmdTorqueBodyMsgPayload.h>
#include <architecture/msgPayloadDef/DipoleRequestBodyMsgPayload.h>
#include <architecture/msgPayloadDef/TAMSensorBodyMsgPayload.h>
#include <architecture/utilities/bskLogging.h>
#include <stdint.h>

/*! @brief Top level structure for the sub-module routines. */
class Torque2Dipole : public SysModel {
   public:
    void reset(uint64_t callTime) override;
    void updateState(uint64_t callTime) override;

    /* Inputs.*/
    ReadFunctor<TAMSensorBodyMsgPayload>
        tamSensorBodyInMsg;  //!< [Tesla] input message for magnetic field sensor data in the Body frame
    ReadFunctor<CmdTorqueBodyMsgPayload>
        tauRequestInMsg;  //!< [N-m] input message containing control torque in the Body frame

    /* Outputs.*/
    Message<DipoleRequestBodyMsgPayload>
        dipoleRequestOutMsg;  //!< [A-m2] output message containing dipole request in the Body frame

    /* Other. */
    BSKLogger bskLogger = {};  //!< BSK Logging
};

#endif
