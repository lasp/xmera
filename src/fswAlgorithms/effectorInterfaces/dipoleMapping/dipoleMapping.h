// SPDX-License-Identifier: ISC
// Copyright (c) 2021, Autonomous Vehicle System Lab, University of Colorado at Boulder
// Copyright (c) 2025, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

#ifndef DIPOLEMAPPING_H
#define DIPOLEMAPPING_H

#include <architecture/_GeneralModuleFiles/sys_model.h>
#include <architecture/messaging/messaging.h>
#include <architecture/msgPayloadDef/DipoleRequestBodyMsgPayload.h>
#include <architecture/msgPayloadDef/MTBArrayConfigMsgPayload.h>
#include <architecture/msgPayloadDef/MTBCmdMsgPayload.h>
#include <architecture/utilities/bskLogging.h>
#include <stdint.h>

/*! @brief Top level structure for the sub-module routines. */
class DipoleMapping : public SysModel {
   public:
    void reset(uint64_t callTime) override;
    void updateState(uint64_t callTime) override;

    /* Configs.*/
    double steeringMatrix[MAX_EFF_CNT *
                          3];  //!< matrix for mapping body frame dipole request to individual torque bar dipoles

    /* Inputs. */
    ReadFunctor<MTBArrayConfigMsgPayload>
        mtbArrayConfigParamsInMsg;  //!< input message containing configuration parameters for all the torque bars on
                                    //!< the vehicle
    ReadFunctor<DipoleRequestBodyMsgPayload>
        dipoleRequestBodyInMsg;  //!< [A-m2] input message containing the requested body frame dipole

    /* Outputs. */
    Message<MTBCmdMsgPayload> dipoleRequestMtbOutMsg;  //!< [A-m2] output message containing the individual dipole
                                                       //!< requests for each torque bar on the vehicle

    /* Other. */
    MTBArrayConfigMsgPayload
        mtbArrayConfigParams;  //!< configuration parameters for all the torque bars used on the vehicle
    BSKLogger bskLogger = {};  //!< BSK Logging
};

#endif
