// SPDX-License-Identifier: ISC
// Copyright (c) 2015, Autonomous Vehicle System Lab, University of Colorado at Boulder
// Copyright (c) 2025, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

#ifndef MRP_FEEDBACK_CONTROL_C_H
#define MRP_FEEDBACK_CONTROL_C_H

#include <stdint.h>

#include <architecture/_GeneralModuleFiles/sys_model.h>
#include <architecture/messaging/messaging.h>
#include <architecture/msgPayloadDef/AttGuidMsgPayload.h>
#include <architecture/msgPayloadDef/CmdTorqueBodyMsgPayload.h>
#include <architecture/msgPayloadDef/RWArrayConfigMsgPayload.h>
#include <architecture/msgPayloadDef/RWAvailabilityMsgPayload.h>
#include <architecture/msgPayloadDef/RWSpeedMsgPayload.h>
#include <architecture/msgPayloadDef/VehicleConfigMsgPayload.h>

/*! @brief Data configuration structure for the MRP feedback attitude control routine. */
class MrpFeedback_C : public SysModel {
   public:
    void reset(uint64_t callTime) override;
    void updateState(uint64_t callTime) override;

    double K;                     //!< [rad/sec] Proportional gain applied to MRP errors
    double P;                     //!< [N*m*s]   Rate error feedback gain applied
    double Ki;                    //!< [N*m]     Integration feedback error on rate error
    double integralLimit;         //!< [N*m]     Integration limit to avoid wind-up issue
    int controlLawType;           //!<           Flag to choose between the two control laws available
    uint64_t priorTime;           //!< [ns]      Last time the attitude control is called
    double z[3];                  //!< [rad]     integral state of delta_omega
    double int_sigma[3];          //!< [s]       integral of the MPR attitude error
    double knownTorquePntB_B[3];  //!< [N*m]     known external torque in body frame vector components

    double ISCPntB_B[9];  //!< [kg m^2]  Spacecraft Inertia
    RWArrayConfigMsgPayload
        rwConfigParams;  //!< [-] struct to store message containing RW config parameters in body B frame

    /* declare module IO interfaces */
    ReadFunctor<RWSpeedMsgPayload> rwSpeedsInMsg;        //!< RW speed input message (Optional)
    ReadFunctor<RWAvailabilityMsgPayload> rwAvailInMsg;  //!< RW availability input message (Optional)
    ReadFunctor<RWArrayConfigMsgPayload> rwParamsInMsg;  //!< RW parameter input message.  (Optional)
    Message<CmdTorqueBodyMsgPayload> cmdTorqueOutMsg;  //!< commanded spacecraft external control torque output message
    Message<CmdTorqueBodyMsgPayload>
        intFeedbackTorqueOutMsg;                          //!< commanded integral feedback control torque output message
    ReadFunctor<AttGuidMsgPayload> guidInMsg;             //!< attitude guidance input message
    ReadFunctor<VehicleConfigMsgPayload> vehConfigInMsg;  //!< vehicle configuration input message

    BSKLogger bskLogger = {};  //!< BSK Logging
};

#endif
