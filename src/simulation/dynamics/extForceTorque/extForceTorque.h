// SPDX-License-Identifier: ISC
// Copyright (c) 2016, Autonomous Vehicle System Lab, University of Colorado at Boulder
// Copyright (c) 2025, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

#ifndef EXT_FORCE_TORQUE_H
#define EXT_FORCE_TORQUE_H

#include <architecture/_GeneralModuleFiles/sys_model.h>
#include <architecture/messaging/messaging.h>
#include <simulation/dynamics/_GeneralModuleFiles/dynamicEffector.h>

#include <architecture/msgPayloadDef/CmdForceBodyMsgPayload.h>
#include <architecture/msgPayloadDef/CmdForceInertialMsgPayload.h>
#include <architecture/msgPayloadDef/CmdTorqueBodyMsgPayload.h>

#include <architecture/utilities/bskLogging.h>

/*! @brief external force and torque dynamic efector class */
class ExtForceTorque : public SysModel, public DynamicEffector {
   public:
    ExtForceTorque();
    ~ExtForceTorque();

    void reset(uint64_t currentSimNanos);
    void updateState(uint64_t currentSimNanos);       //!< class method
    void linkInStates(DynParamManager& statesIn);     //!< class method
    void writeOutputMessages(uint64_t currentClock);  //!< class method
    void readInputMessages();
    void computeForceTorque(double integTime, double timeStep);

   private:
    CmdTorqueBodyMsgPayload incomingCmdTorqueBuffer;            //!< -- One-time allocation for savings
    CmdForceInertialMsgPayload incomingCmdForceInertialBuffer;  //!< -- One-time allocation for savings
    CmdForceBodyMsgPayload incomingCmdForceBodyBuffer;          //!< -- One-time allocation for savings

   public:
    Eigen::Vector3d extForce_N;       //!< [N]  external force in inertial  frame components
    Eigen::Vector3d extForce_B;       //!< [N]  external force in body frame components
    Eigen::Vector3d extTorquePntB_B;  //!< [Nm] external torque in body frame components

    BSKLogger bskLogger;                                            //!< -- BSK Logging
    ReadFunctor<CmdTorqueBodyMsgPayload> cmdTorqueInMsg;            //!< commanded torque input msg
    ReadFunctor<CmdForceBodyMsgPayload> cmdForceBodyInMsg;          //!< commanded force input msg in B frame
    ReadFunctor<CmdForceInertialMsgPayload> cmdForceInertialInMsg;  //!< commanded force input msg in N frame
};

#endif
