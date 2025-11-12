#ifndef _PRV_STEERING_CONTROL_H_
#define _PRV_STEERING_CONTROL_H_

#include <architecture/_GeneralModuleFiles/sys_model.h>
#include <architecture/messaging/messaging.h>
#include <architecture/msgPayloadDef/AttGuidMsgPayload.h>
#include <architecture/msgPayloadDef/RateCmdMsgPayload.h>
#include <stdint.h>

/*! module configuration message definition */
class PrvSteering : public SysModel {
   public:
    void reset(uint64_t callTime) override;
    void updateState(uint64_t callTime) override;

    /* declare module private variables */
    double K1;        /*!< [rad/sec] Proportional gain applied to principal rotation angle error */
    double K3;        /*!< [rad/sec] Cubic gain applied to principal rotation angle error
                          in steering saturation function */
    double omega_max; /*!< [rad/sec] Maximum rate command of steering control */

    /* declare module IO interfaces */
    Message<RateCmdMsgPayload> rateCmdOutMsg;  //!< rate command output message
    ReadFunctor<AttGuidMsgPayload> guidInMsg;  //!< attitude guidance input message

    BSKLogger bskLogger = {};  //!< BSK Logging
};

void PRVSteeringLaw(PrvSteering* configData, double sigma_BR[3], double omega_ast[3], double omega_ast_p[3]);

#endif
