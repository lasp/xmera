/*
 ISC License

 Copyright (c) 2016, Autonomous Vehicle Systems Lab, University of Colorado at Boulder

 Permission to use, copy, modify, and/or distribute this software for any
 purpose with or without fee is hereby granted, provided that the above
 copyright notice and this permission notice appear in all copies.

 THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
 WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
 MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
 ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
 WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
 ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
 OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.

 */

#ifndef _RATE_SERVO_FULL_NONLINEAR_
#define _RATE_SERVO_FULL_NONLINEAR_

#include "architecture/_GeneralModuleFiles/sys_model.h"
#include "architecture/messaging/messaging.h"
#include "architecture/msgPayloadDef/AttGuidMsgPayload.h"
#include "architecture/msgPayloadDef/CmdTorqueBodyMsgPayload.h"
#include "architecture/msgPayloadDef/RWArrayConfigMsgPayload.h"
#include "architecture/msgPayloadDef/RWAvailabilityMsgPayload.h"
#include "architecture/msgPayloadDef/RWSpeedMsgPayload.h"
#include "architecture/msgPayloadDef/RateCmdMsgPayload.h"
#include "architecture/msgPayloadDef/VehicleConfigMsgPayload.h"

#include <stdint.h>

#include <Eigen/Core>

/*! @brief The configuration structure for the rateServoFullNonlinear module.  */
class RateServoFullNonlinear : public SysModel {
   public:
    void reset(uint64_t callTime) override;
    void updateState(uint64_t callTime) override;

    void setP(const double gain);
    double getP() const;
    void setKi(const double gain);
    double getKi() const;
    void setIntegralLimit(const double limit);
    double getIntegralLimit() const;
    void setKnownTorquePntB_B(const Eigen::Vector3d &knownTorquePntB_B);
    Eigen::Vector3d getKnownTorquePntB_B() const;

    /* declare module IO interfaces */
    Message<CmdTorqueBodyMsgPayload> cmdTorqueOutMsg;     //!< commanded torque output message
    ReadFunctor<AttGuidMsgPayload> guidInMsg;             //!< attitude guidance input message
    ReadFunctor<VehicleConfigMsgPayload> vehConfigInMsg;  //!< vehicle configuration input message
    ReadFunctor<RWSpeedMsgPayload> rwSpeedsInMsg;         //!< (optional) RW speed input message
    ReadFunctor<RWAvailabilityMsgPayload> rwAvailInMsg;   //!< (optional) RW availability input message
    ReadFunctor<RWArrayConfigMsgPayload> rwParamsInMsg;   //!< (optional) RW configuration parameter input message
    ReadFunctor<RateCmdMsgPayload> rateSteeringInMsg;     //!< commanded rate input message

   private:
    double P{};                     //!< [N*m*s]   Rate error feedback gain applied
    double Ki{};                    //!< [N*m]     Integration feedback error on rate error
    double integralLimit{};         //!< [N*m]     Integration limit to avoid wind-up issue
    Eigen::Vector3d knownTorquePntB_B{Eigen::Vector3d::Zero()};  //!< [N*m]     known external torque in body frame vector components
    uint64_t priorTime{};   //!< [ns]      Last time the attitude control is called
    Eigen::Vector3d z{};          //!< [rad]     integral state of delta_omega
    Eigen::Matrix3d ISCPntB_B{};  //!< [kg m^2] Spacecraft Inertia
    RWArrayConfigMsgPayload
        rwConfigParams{};  //!< [-] struct to store message containing RW config parameters in body B frame
};

#endif
