#ifndef _ET_SPHERICAL_CONTROL_H_
#define _ET_SPHERICAL_CONTROL_H_

#include <architecture/_GeneralModuleFiles/sys_model.h>
#include <architecture/messaging/messaging.h>
#include <architecture/msgPayloadDef/CmdForceBodyMsgPayload.h>
#include <architecture/msgPayloadDef/CmdForceInertialMsgPayload.h>
#include <architecture/msgPayloadDef/NavAttMsgPayload.h>
#include <architecture/msgPayloadDef/NavTransMsgPayload.h>
#include <architecture/msgPayloadDef/VehicleConfigMsgPayload.h>
#include <stdint.h>

#include "architecture/utilities/bskLogging.h"

/*! @brief Top level structure for the sub-module routines. */
class EtSphericalControl : public SysModel {
   public:
    void reset(uint64_t callTime) override;
    void updateState(uint64_t callTime) override;
    void calcRelativeMotionControl(NavTransMsgPayload servicerTransInMsgBuffer,
                                   NavTransMsgPayload debrisTransInMsgBuffer,
                                   NavAttMsgPayload servicerAttInMsgBuffer,
                                   VehicleConfigMsgPayload servicerVehicleConfigInMsgBuffer,
                                   VehicleConfigMsgPayload debrisVehicleConfigInMsgBuffer,
                                   CmdForceInertialMsgPayload eForceInMsgBuffer,
                                   CmdForceInertialMsgPayload* forceInertialOutMsgBuffer,
                                   CmdForceBodyMsgPayload* forceBodyOutMsgBuffer);
    // declare module IO interfaces
    ReadFunctor<NavTransMsgPayload> servicerTransInMsg;  //!< servicer orbit input message
    ReadFunctor<NavTransMsgPayload> debrisTransInMsg;    //!< debris orbit input message
    ReadFunctor<NavAttMsgPayload> servicerAttInMsg;      //!< servicer attitude input message
    ReadFunctor<VehicleConfigMsgPayload>
        servicerVehicleConfigInMsg;  //!< servicer vehicle configuration (mass) input message
    ReadFunctor<VehicleConfigMsgPayload>
        debrisVehicleConfigInMsg;                             //!< debris vehicle configuration (mass) input message
    ReadFunctor<CmdForceInertialMsgPayload> eForceInMsg;      //!< servicer electrostatic force input message
    Message<CmdForceInertialMsgPayload> forceInertialOutMsg;  //!< servicer inertial frame control force output message
    Message<CmdForceBodyMsgPayload> forceBodyOutMsg;          //!< servicer body frame control force output message

    double mu;                 //!< [m^3/s^2]  gravitational parameter
    double L_r;                //!< [m]  reference separation distance
    double theta_r;            //!< [rad]  reference in-plane rotation angle
    double phi_r;              //!< [rad]  reference out-of-plane rotation angle
    double K[9];               //!< 3x3 symmetric positive definite feedback gain matrix [K]
    double P[9];               //!< 3x3 symmetric positive definite feedback gain matrix [P]
    BSKLogger bskLogger = {};  //!< BSK Logging
};

#endif
