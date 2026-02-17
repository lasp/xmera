// SPDX-License-Identifier: ISC
// Copyright (c) 2016, Autonomous Vehicle System Lab, University of Colorado at Boulder
// Copyright (c) 2025, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

#ifndef _ST_COMM_H_
#define _ST_COMM_H_

#include <architecture/_GeneralModuleFiles/sys_model.h>
#include <architecture/messaging/messaging.h>
#include <architecture/msgPayloadDef/STAttMsgPayload.h>
#include <architecture/msgPayloadDef/STSensorMsgPayload.h>

#include <architecture/utilities/bskLogging.h>

/*! @brief Module configuration message.  */
class StComm : public SysModel {
   public:
    void reset(uint64_t callTime) override;
    void updateState(uint64_t callTime) override;

    double dcm_BP[9];                              /*!< Row major platform 2 body DCM*/
    ReadFunctor<STSensorMsgPayload> stSensorInMsg; /*!< star tracker sensor input message*/
    Message<STAttMsgPayload> stAttOutMsg;          /*!< star tracker attitude output message */

    BSKLogger bskLogger{};  //!< BSK Logging
};

#endif
