#ifndef _HINGED_RIGID_BODY_PID_MOTOR_
#define _HINGED_RIGID_BODY_PID_MOTOR_

#include <architecture/_GeneralModuleFiles/sys_model.h>
#include <architecture/messaging/messaging.h>
#include <architecture/msgPayloadDef/ArrayMotorTorqueMsgPayload.h>
#include <architecture/msgPayloadDef/HingedRigidBodyMsgPayload.h>
#include <architecture/utilities/bskLogging.h>
#include <stdint.h>

/*! @brief Top level structure for the sub-module routines. */
class HingedRigidBodyPIDMotor : public SysModel {
   public:
    void reset(uint64_t callTime) override;
    void updateState(uint64_t callTime) override;

    /*! declare these user-defined input parameters */
    double K;  //!< proportional gain
    double P;  //!< derivative gain
    double I;  //!< integral gain

    /*! declare these variables for internal computations */
    uint64_t priorTime;      //!< prior function call time for trapezoid integration
    double priorThetaError;  //!< theta error at prior function call
    double intError;         //!< integral error

    /* declare module IO interfaces */
    ReadFunctor<HingedRigidBodyMsgPayload> hingedRigidBodyInMsg;  //!< input spinning body message
    ReadFunctor<HingedRigidBodyMsgPayload>
        hingedRigidBodyRefInMsg;  //!< output msg containing spinning body target angle and angle rate
    Message<ArrayMotorTorqueMsgPayload>
        motorTorqueOutMsg;  //!< output msg containing the motor torque to the array drive

    BSKLogger bskLogger = {};  //!< BSK Logging
};

#endif
