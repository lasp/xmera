// SPDX-License-Identifier: ISC
// Copyright (c) 2024, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

#ifndef SUN_SEARCH_H
#define SUN_SEARCH_H

#include <architecture/_GeneralModuleFiles/sys_model.h>
#include <architecture/messaging/messaging.h>
#include <architecture/msgPayloadDef/AttGuidMsgPayload.h>
#include <architecture/msgPayloadDef/NavAttMsgPayload.h>
#include <architecture/msgPayloadDef/VehicleConfigMsgPayload.h>
#include "sunSearchAlgorithm.h"

class SunSearch : public SysModel {
   public:
    SunSearch() = default;
    ~SunSearch() = default;

    void reset(uint64_t currentSimNanos);
    void updateState(uint64_t currentSimNanos);
    void setSlewProperties(SlewProperties slewPropertiesInput);
    void modifySlewProperties(SlewProperties slewPropertiesInput, uint32_t index);
    SlewProperties getSlewProperties(uint32_t index) const;

    ReadFunctor<NavAttMsgPayload> attNavInMsg;            //!< input msg measured attitude
    ReadFunctor<VehicleConfigMsgPayload> vehConfigInMsg;  //!< input veh config msg
    Message<AttGuidMsgPayload> attGuidOutMsg;             //!< Attitude reference output message

   private:
    SunSearchAlgorithm algorithm{};
};

#endif
