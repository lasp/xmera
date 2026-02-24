// SPDX-License-Identifier: ISC
// Copyright (c) 2024, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

#ifndef _THR_MOMENTUM_MANAGEMENT_CPP_H_
#define _THR_MOMENTUM_MANAGEMENT_CPP_H_

#include <architecture/_GeneralModuleFiles/sys_model.h>
#include <architecture/messaging/messaging.h>
#include <architecture/utilities/bskLogging.h>

#include <architecture/msgPayloadDef/CmdTorqueBodyMsgPayload.h>
#include <architecture/msgPayloadDef/RWArrayConfigMsgPayload.h>
#include <architecture/msgPayloadDef/RWSpeedMsgPayload.h>

#include <Eigen/Core>

/*! @brief Module configuration message definition. */
class ThrMomentumManagementCpp : public SysModel {
   public:
    void reset(uint64_t currentSimNanos) override;        //!< Reset function
    void updateState(uint64_t currentSimNanos) override;  //!< Update function

    double hs_min = 0;  //!< [Nms]  minimum RW cluster momentum for dumping
    Eigen::Vector3d hd_B;
    RWArrayConfigMsgPayload
        rwConfigParams;  //!< [-] struct to store message containing RW config parameters in body B frame

    Message<CmdTorqueBodyMsgPayload> deltaHOutMsg = {};      //!< The name of the output message
    ReadFunctor<RWSpeedMsgPayload> rwSpeedsInMsg;            //!< [] The name for the reaction wheel speeds message
    ReadFunctor<RWArrayConfigMsgPayload> rwConfigDataInMsg;  //!< [-] The name of the RWA configuration message

    BSKLogger* bskLogger;  //!< BSK Logging
   private:
    int initRequest = 1;  //!< [-] status flag of the momentum dumping management
};

#endif
