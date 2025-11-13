#ifndef _THRUSTER_PLATFORM_STATE_
#define _THRUSTER_PLATFORM_STATE_

#include <architecture/_GeneralModuleFiles/sys_model.h>
#include <architecture/messaging/messaging.h>
#include <architecture/msgPayloadDef/CmdTorqueBodyMsgPayload.h>
#include <architecture/msgPayloadDef/HingedRigidBodyMsgPayload.h>
#include <architecture/msgPayloadDef/THRConfigMsgPayload.h>
#include <architecture/utilities/bskLogging.h>
#include <stdint.h>

/*! @brief Top level structure for the sub-module routines. */
class ThrusterPlatformState : public SysModel {
   public:
    void reset(uint64_t callTime) override;
    void updateState(uint64_t callTime) override;

    /* declare these user-defined quantities */
    double sigma_MB[3];  //!< orientation of the M frame w.r.t. the B frame
    double r_BM_M[3];    //!< position of B frame origin w.r.t. M frame origin, in M frame coordinates
    double r_FM_F[3];    //!< position of F frame origin w.r.t. M frame origin, in F frame coordinates

    double K;  //!< momentum dumping time constant [1/s]

    /* declare module IO interfaces */
    ReadFunctor<THRConfigMsgPayload> thrusterConfigFInMsg;  //!< input thruster configuration msg
    ReadFunctor<HingedRigidBodyMsgPayload>
        hingedRigidBody1InMsg;  //!< output msg containing theta1 reference and thetaDot1 reference
    ReadFunctor<HingedRigidBodyMsgPayload>
        hingedRigidBody2InMsg;  //!< output msg containing theta2 reference and thetaDot2 reference
    Message<THRConfigMsgPayload>
        thrusterConfigBOutMsg;  //!< output msg containing the thruster configuration infor in B-frame

    BSKLogger bskLogger = {};  //!< BSK Logging
};

#endif
