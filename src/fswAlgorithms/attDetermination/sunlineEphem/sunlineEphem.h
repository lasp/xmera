// SPDX-License-Identifier: ISC
// Copyright (c) 2016, Autonomous Vehicle System Lab, University of Colorado at Boulder
// Copyright (c) 2025, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

#ifndef _SUNLINE_EPHEM_H_
#define _SUNLINE_EPHEM_H_

#include <architecture/_GeneralModuleFiles/sys_model.h>
#include <architecture/messaging/messaging.h>
#include "sunlineEphemAlgorithm.h"

class SunlineEphem : public SysModel {
   public:
    SunlineEphem();
    void reset(uint64_t callTime) override;
    void updateState(uint64_t callTime) override;
    Message<NavAttMsgPayload> navStateOutMsg;           /*!< The name of the output message*/
    ReadFunctor<EphemerisMsgPayload> sunPositionInMsg;  //!< The name of the sun ephemeris input message
    ReadFunctor<NavTransMsgPayload> scPositionInMsg;    //!< The name of the spacecraft ephemeris input message
    ReadFunctor<NavAttMsgPayload> scAttitudeInMsg;      //!< The name of the spacecraft attitude input message

   private:
    SunlineEphemAlgorithm algorithm;
};

#endif
