// SPDX-License-Identifier: ISC
// Copyright (c) 2021, Autonomous Vehicle System Lab, University of Colorado at Boulder
// Copyright (c) 2025, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

#ifndef MTBEFFECTOR_H
#define MTBEFFECTOR_H

#include <architecture/_GeneralModuleFiles/sys_model.h>
#include <architecture/msgPayloadDef/MTBArrayConfigMsgPayload.h>
#include <architecture/msgPayloadDef/MTBCmdMsgPayload.h>
#include <architecture/msgPayloadDef/MTBMsgPayload.h>
#include <architecture/msgPayloadDef/MagneticFieldMsgPayload.h>
#include <architecture/utilities/eigenMRP.h>
#include <simulation/dynamics/_GeneralModuleFiles/dynamicEffector.h>
#include <simulation/dynamics/_GeneralModuleFiles/stateData.h>

#include <architecture/messaging/messaging.h>
#include <architecture/utilities/bskLogging.h>

/*! @brief This module converts magnetic torque bar dipoles to body torques.
 */
class MtbEffector : public SysModel, public DynamicEffector {
   public:
    MtbEffector();
    ~MtbEffector();
    void reset(uint64_t currentSimNanos);
    void updateState(uint64_t currentSimNanos);
    void linkInStates(DynParamManager& states);
    void computeForceTorque(double integTime, double timeStep);
    void WriteOutputMessages(uint64_t CurrentClock);

   public:
    Message<MTBMsgPayload>
        mtbOutMsg;        //!< output message containing net torque produced by the torque bars in Body components
    StateData* hubSigma;  //!< Hub/Inertial attitude represented by MRP
    ReadFunctor<MTBCmdMsgPayload>
        mtbCmdInMsg;  //!< input msg for commanded mtb dipole array in the magnetic torque bar frame T
    ReadFunctor<MagneticFieldMsgPayload> magInMsg;         //!< input msg for magnetic field data in inertial frame N
    ReadFunctor<MTBArrayConfigMsgPayload> mtbParamsInMsg;  //!< input msg for layout of magnetic torque bars
    BSKLogger bskLogger;                                   //!< -- BSK Logging

   private:
    MTBCmdMsgPayload
        mtbCmdInMsgBuffer;  //!< msg buffer or commanded mtb dipole array in the magnetic torque bar frame T
    MagneticFieldMsgPayload magInMsgBuffer;    //!< msg buffer for magnetic field data in inertial frame N
    MTBArrayConfigMsgPayload mtbConfigParams;  //!< msg for layout of magnetic torque bars
};

#endif
