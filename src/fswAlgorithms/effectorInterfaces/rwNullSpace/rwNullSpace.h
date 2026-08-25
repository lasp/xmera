// SPDX-License-Identifier: ISC
// Copyright (c) 2016, Autonomous Vehicle System Lab, University of Colorado at Boulder
// Copyright (c) 2025, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

#ifndef RW_NULL_SPACE_H
#define RW_NULL_SPACE_H

#include "rwNullSpaceAlgorithm.h"
#include <architecture/_GeneralModuleFiles/sys_model.h>
#include <architecture/messaging/messaging.h>
#include <architecture/msgPayloadDef/RWConstellationMsgPayload.h>
#include <architecture/msgPayloadDef/RwMotorTorqueMsgPayload.h>
#include <architecture/msgPayloadDef/RWSpeedMsgPayload.h>

#include <stdint.h>

/*! @brief The configuration structure for the rwNullSpace module.  */
class RwNullSpace final : public SysModel {
public:
    void reset(uint64_t callTime) override;
    void updateState(uint64_t callTime) override;

    void setOmegaGain(double const gain);
    double getOmegaGain() const;

    ReadFunctor<RwMotorTorqueMsgPayload> rwMotorTorqueInMsg;  //!< [-] The name of the Input message
    ReadFunctor<RWSpeedMsgPayload> rwSpeedsInMsg;             //!< [-] The name of the input RW speeds
    ReadFunctor<RWSpeedMsgPayload> rwDesiredSpeedsInMsg;      //!< [-] (optional) The name of the desired RW speeds
    ReadFunctor<RWConstellationMsgPayload> rwConfigInMsg;     //!< [-] The name of the RWA configuration message
    Message<RwMotorTorqueMsgPayload> rwMotorTorqueOutMsg;     //!< [-] The name of the output message

private:
    RwNullSpaceAlgorithm algorithm{};
};

#endif
