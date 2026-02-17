// SPDX-License-Identifier: ISC
// Copyright (c) 2016, Autonomous Vehicle System Lab, University of Colorado at Boulder
// Copyright (c) 2025, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

#ifndef _SIMPLE_DEADBAND_
#define _SIMPLE_DEADBAND_

#include <architecture/_GeneralModuleFiles/sys_model.h>
#include <architecture/messaging/messaging.h>
#include <architecture/msgPayloadDef/AttGuidMsgPayload.h>
#include <architecture/utilities/bskLogging.h>
#include <stdint.h>

/*! @brief Top level structure for the sub-module routines. */
class SimpleDeadband : public SysModel {
   public:
    void reset(uint64_t callTime) override;
    void updateState(uint64_t callTime) override;
    void applyDBLogic_simpleDeadband();

    /* declare module private variables */
    double innerAttThresh;  /*!< inner limit for sigma (attitude) errors */
    double outerAttThresh;  /*!< outer limit for sigma (attitude) errors */
    double innerRateThresh; /*!< inner limit for omega (rate) errors */
    double outerRateThresh; /*!< outer limit for omega (rate) errors */
    uint32_t wasControlOff; /*!< boolean variable to keep track of the last Control status (ON/OFF) */
    double attError;        /*!< current scalar attitude error */
    double rateError;       /*!< current scalar rate error */

    /* declare module IO interfaces */
    Message<AttGuidMsgPayload> attGuidOutMsg; /*!< The name of the output message*/
    ReadFunctor<AttGuidMsgPayload> guidInMsg; /*!< The name of the guidance reference Input message */

    AttGuidMsgPayload attGuidOut; /*!< copy of the output message */
    BSKLogger bskLogger = {};     //!< BSK Logging
};

#endif
