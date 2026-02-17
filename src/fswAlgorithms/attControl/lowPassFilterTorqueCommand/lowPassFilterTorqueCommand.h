// SPDX-License-Identifier: ISC
// Copyright (c) 2016, Autonomous Vehicle System Lab, University of Colorado at Boulder
// Copyright (c) 2025, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

#ifndef _LOW_PASS_FILTER_TORQUE_COMMAND_
#define _LOW_PASS_FILTER_TORQUE_COMMAND_

#include <architecture/_GeneralModuleFiles/sys_model.h>
#include <architecture/messaging/messaging.h>
#include <architecture/msgPayloadDef/CmdTorqueBodyMsgPayload.h>
#include <stdint.h>

#define NUM_LPF 2 /*            number of states to track, including current state */

/*! @brief module configuration message. */
class LowPassFilterTorqueCommand : public SysModel {
   public:
    void reset(uint64_t callTime) override;
    void updateState(uint64_t callTime) override;

    /* declare module private variables */
    double h;               /*!< [s]      filter time step (assumed to be fixed */
    double wc;              /*!< [rad/s]  continuous filter cut-off frequency */
    double hw;              /*!< [rad]    h times the prewarped discrete time cut-off frequency */
    double a[NUM_LPF];      /*!<          filter coefficients for output */
    double b[NUM_LPF];      /*!<          filter coefficients for input */
    double Lr[NUM_LPF][3];  /*!< [Nm]     prior torque command */
    double LrF[NUM_LPF][3]; /*!< [Nm]     prior filtered torque command */
    int shouldBeReset;      /*!<          flag indicating the filter being started up */

    /* declare module IO interfaces */
    Message<CmdTorqueBodyMsgPayload> cmdTorqueOutMsg;     //!< commanded torque output message
    ReadFunctor<CmdTorqueBodyMsgPayload> cmdTorqueInMsg;  //!< commanded torque input message

    BSKLogger bskLogger = {};  //!< BSK Logging
};

#endif
