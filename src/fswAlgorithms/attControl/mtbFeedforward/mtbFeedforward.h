#ifndef MTBFEEDFORWARD_H
#define MTBFEEDFORWARD_H

#include <architecture/_GeneralModuleFiles/sys_model.h>
#include <architecture/messaging/messaging.h>
#include <architecture/msgPayloadDef/CmdTorqueBodyMsgPayload.h>
#include <architecture/msgPayloadDef/MTBArrayConfigMsgPayload.h>
#include <architecture/msgPayloadDef/MTBCmdMsgPayload.h>
#include <architecture/msgPayloadDef/TAMSensorBodyMsgPayload.h>
#include <stdint.h>

/*! @brief Top level structure for the sub-module routines. */
class MtbFeedforward : public SysModel {
   public:
    void reset(uint64_t callTime) override;
    void updateState(uint64_t callTime) override;

    /* Inputs. */
    ReadFunctor<CmdTorqueBodyMsgPayload>
        vehControlInMsg;  //!< input message containing the current control torque in the Body frame
    ReadFunctor<MTBCmdMsgPayload> dipoleRequestMtbInMsg;  //!< input message containing the individual dipole requests
                                                          //!< for each torque bar on the vehicle
    ReadFunctor<TAMSensorBodyMsgPayload>
        tamSensorBodyInMsg;  //!< [Tesla] input message for magnetic field sensor data in the Body frame
    ReadFunctor<MTBArrayConfigMsgPayload>
        mtbArrayConfigParamsInMsg;  //!< input message containing configuration parameters for all the torque bars on
                                    //!< the vehicle

    /* Outputs. */
    Message<CmdTorqueBodyMsgPayload>
        vehControlOutMsg;  //!< output message containing the current control torque in the Body frame

    /* Other. */
    MTBArrayConfigMsgPayload mtbArrayConfigParams;  //!< configuration for MTB layout
    BSKLogger bskLogger = {};                       //!< BSK Logging
};

#endif
