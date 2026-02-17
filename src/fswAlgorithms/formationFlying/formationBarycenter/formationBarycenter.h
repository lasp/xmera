// SPDX-License-Identifier: ISC
// Copyright (c) 2021, Autonomous Vehicle System Lab, University of Colorado at Boulder
// Copyright (c) 2025, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

#ifndef FORMATION_BARYCENTER_H
#define FORMATION_BARYCENTER_H

#include <vector>

#include <architecture/msgPayloadDef/NavTransMsgPayload.h>
#include <architecture/msgPayloadDef/VehicleConfigMsgPayload.h>

#include <architecture/_GeneralModuleFiles/sys_model.h>
#include <architecture/messaging/messaging.h>
#include <architecture/utilities/bskLogging.h>

/*! @brief This module computes the barycenter of a swarm of satellites, either using cartesian coordinates or orbital
 * elements.
 */
class FormationBarycenter : public SysModel {
   public:
    FormationBarycenter();

    void reset(uint64_t currentSimNanos);
    void updateState(uint64_t currentSimNanos);
    void ReadInputMessages();
    void addSpacecraftToModel(Message<NavTransMsgPayload>* tmpScNavMsg,
                              Message<VehicleConfigMsgPayload>* tmpScPayloadMsg);
    void computeBaricenter();
    void WriteOutputMessage(uint64_t CurrentClock);

   public:
    std::vector<ReadFunctor<NavTransMsgPayload>> scNavInMsgs;           //!< spacecraft navigation input msg
    std::vector<ReadFunctor<VehicleConfigMsgPayload>> scPayloadInMsgs;  //!< spacecraft payload input msg

    Message<NavTransMsgPayload> transOutMsg;  //!< translation navigation output msg

    bool useOrbitalElements;  //!< flag that determines whether to use cartesian or orbital elementd weighted averaging
    double mu;                //!< gravitational parameter to be used with orbital elements averaging

    BSKLogger bskLogger;  //!< -- BSK Logging

   private:
    std::vector<NavTransMsgPayload> scNavBuffer;           //!< buffer of spacecraft navigation info
    std::vector<VehicleConfigMsgPayload> scPayloadBuffer;  //!< buffer of spacecraft payload

    NavTransMsgPayload transOutBuffer;  //!< buffer for the output message
};

#endif
