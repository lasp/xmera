#ifndef _THR_MOMENTUM_MANAGEMENT_H_
#define _THR_MOMENTUM_MANAGEMENT_H_

#include <stdint.h>

#include <architecture/_GeneralModuleFiles/sys_model.h>
#include <architecture/messaging/messaging.h>
#include <architecture/msgPayloadDef/CmdTorqueBodyMsgPayload.h>
#include <architecture/msgPayloadDef/RWArrayConfigMsgPayload.h>
#include <architecture/msgPayloadDef/RWSpeedMsgPayload.h>

/*! @brief Module configuration message definition. */
class ThrMomentumManagement : public SysModel {
   public:
    void reset(uint64_t callTime) override;
    void updateState(uint64_t callTime) override;

    /* declare module private variables */
    int initRequest;  //!< [-] status flag of the momentum dumping management
    RWArrayConfigMsgPayload
        rwConfigParams;  //!< [-] struct to store message containing RW config parameters in body B frame

    /* declare module public variables */
    double hs_min;  //!< [Nms]  minimum RW cluster momentum for dumping

    /* declare module IO interfaces */
    Message<CmdTorqueBodyMsgPayload> deltaHOutMsg;           //!< The name of the output message
    ReadFunctor<RWSpeedMsgPayload> rwSpeedsInMsg;            //!< [] The name for the reaction wheel speeds message
    ReadFunctor<RWArrayConfigMsgPayload> rwConfigDataInMsg;  //!< [-] The name of the RWA configuration message

    BSKLogger bskLogger = {};  //!< BSK Logging
};

#endif
