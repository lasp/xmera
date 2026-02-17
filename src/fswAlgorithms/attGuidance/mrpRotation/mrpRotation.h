// SPDX-License-Identifier: ISC
// Copyright (c) 2016, Autonomous Vehicle System Lab, University of Colorado at Boulder
// Copyright (c) 2025, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

#ifndef MRP_ROTATION_H
#define MRP_ROTATION_H

#include <stdint.h>

#include <architecture/_GeneralModuleFiles/sys_model.h>
#include <architecture/messaging/messaging.h>
#include <architecture/msgPayloadDef/AttRefMsgPayload.h>
#include <architecture/msgPayloadDef/AttStateMsgPayload.h>
#include "mrpRotationAlgorithm.h"
#include <Eigen/Core>

/*! @brief MRP Rotation class */
class MrpRotation : public SysModel {
   public:
    MrpRotation() = default;
    ~MrpRotation() final = default;

    void reset(uint64_t callTime) override;
    void updateState(uint64_t callTime) override;

    void setSigmaRR0(const Eigen::Vector3d& sigma);
    const Eigen::Vector3d getSigmaRR0() const;
    void setOmegaRR0(const Eigen::Vector3d& omega);
    const Eigen::Vector3d getOmegaRR0() const;

    Message<AttRefMsgPayload> attRefOutMsg;           //!< output message containing the Reference
    ReadFunctor<AttRefMsgPayload> attRefInMsg;        //!< guidance reference input message
    ReadFunctor<AttStateMsgPayload> desiredAttInMsg;  //!< incoming message containing the desired attitude set

   private:
    MrpRotationAlgorithm algorithm{};
};

#endif
