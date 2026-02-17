// SPDX-License-Identifier: ISC
// Copyright (c) 2021, Autonomous Vehicle System Lab, University of Colorado at Boulder
// Copyright (c) 2025, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

#ifndef SIMPLE_MASS_PROPS_H
#define SIMPLE_MASS_PROPS_H

#include <architecture/_GeneralModuleFiles/sys_model.h>
#include <architecture/messaging/messaging.h>
#include <architecture/msgPayloadDef/SCMassPropsMsgPayload.h>
#include <architecture/msgPayloadDef/VehicleConfigMsgPayload.h>
#include <architecture/utilities/bskLogging.h>

/*! @brief FSW mass properties converter module class */
class SimpleMassProps : public SysModel {
   public:
    SimpleMassProps();
    ~SimpleMassProps();

    void reset(uint64_t currentSimNanos);
    void readInputMessages();
    void writeOutputMessages(uint64_t CurrentClock);
    void computeMassProperties();
    void updateState(uint64_t currentSimNanos);

   public:
    ReadFunctor<SCMassPropsMsgPayload> scMassPropsInMsg;   //!< sc mass properties input msg
    Message<VehicleConfigMsgPayload> vehicleConfigOutMsg;  //!< vehicle configuration output msg

    BSKLogger bskLogger;  //!< -- BSK Logging

   private:
    SCMassPropsMsgPayload scMassPropsMsgBuffer;      //! buffer for the mass properties message
    VehicleConfigMsgPayload vehicleConfigMsgBuffer;  //! buffer for the vehicle configuration message
};

#endif
