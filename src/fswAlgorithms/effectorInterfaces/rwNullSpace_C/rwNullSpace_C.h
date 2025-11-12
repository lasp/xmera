#ifndef RW_NULL_SPACE_C_H
#define RW_NULL_SPACE_C_H

#include <architecture/_GeneralModuleFiles/sys_model.h>
#include <architecture/messaging/messaging.h>
#include <architecture/msgPayloadDef/RWConstellationMsgPayload.h>
#include <architecture/msgPayloadDef/RWSpeedMsgPayload.h>
#include <architecture/msgPayloadDef/RwMotorTorqueMsgPayload.h>

#include <architecture/utilities/bskLogging.h>
#include <stdint.h>
#include <stdlib.h>

/*! @brief The configuration structure for the rwNullSpace module.  */
class RwNullSpace_C : public SysModel {
   public:
    void reset(uint64_t callTime) override;
    void updateState(uint64_t callTime) override;
    ReadFunctor<RwMotorTorqueMsgPayload> rwMotorTorqueInMsg;  //!< [-] The name of the Input message
    ReadFunctor<RWSpeedMsgPayload> rwSpeedsInMsg;             //!< [-] The name of the input RW speeds
    ReadFunctor<RWSpeedMsgPayload> rwDesiredSpeedsInMsg;      //!< [-] (optional) The name of the desired RW speeds
    ReadFunctor<RWConstellationMsgPayload> rwConfigInMsg;     //!< [-] The name of the RWA configuration message
    Message<RwMotorTorqueMsgPayload> rwMotorTorqueOutMsg;     //!< [-] The name of the output message

    double tau[RW_EFF_CNT * RW_EFF_CNT];  //!< [-] RW nullspace project matrix
    double OmegaGain;                     //!< [-] The gain factor applied to the RW speeds
    uint32_t numWheels;                   //!< [-] The number of reaction wheels we have

    BSKLogger bskLogger = {};  //!< BSK Logging
};

#endif
