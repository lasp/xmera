#ifndef MRP_STEERING_CONTROL_C_H
#define MRP_STEERING_CONTROL_C_H

#include <architecture/_GeneralModuleFiles/sys_model.h>
#include <architecture/messaging/messaging.h>
#include <architecture/msgPayloadDef/AttGuidMsgPayload.h>
#include <architecture/msgPayloadDef/RateCmdMsgPayload.h>
#include <stdint.h>

/*! @brief Data structure for the MRP feedback attitude control routine. */
class MrpSteering_C : public SysModel {
   public:
    void reset(uint64_t callTime) override;
    void updateState(uint64_t callTime) override;

    /* declare module public variables */
    double K1;         //!< [rad/sec] Proportional gain applied to MRP errors
    double K3;         //!< [rad/sec] Cubic gain applied to MRP error in steering saturation function
    double omega_max;  //!< [rad/sec] Maximum rate command of steering control

    uint32_t
        ignoreOuterLoopFeedforward;  //!< []      Boolean flag indicating if outer feedforward term should be included

    /* declare module IO interfaces */
    Message<RateCmdMsgPayload> rateCmdOutMsg;  //!< rate command output message
    ReadFunctor<AttGuidMsgPayload> guidInMsg;  //!< attitude guidance input message

    BSKLogger bskLogger = {};  //!< BSK Logging
};

#endif
