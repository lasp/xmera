// SPDX-License-Identifier: ISC
// Copyright (c) 2015, Autonomous Vehicle System Lab, University of Colorado at Boulder
// Copyright (c) 2025, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

#ifndef RATE_SERVO_FULL_NONLINEAR_ALGORITHM_H
#define RATE_SERVO_FULL_NONLINEAR_ALGORITHM_H

#include <stdint.h>

#include <architecture/msgPayloadDef/AttGuidMsgPayload.h>
#include <architecture/msgPayloadDef/CmdTorqueBodyMsgPayload.h>
#include <architecture/msgPayloadDef/RWArrayConfigMsgPayload.h>
#include <architecture/msgPayloadDef/RWAvailabilityMsgPayload.h>
#include <architecture/msgPayloadDef/RWSpeedMsgPayload.h>
#include <architecture/msgPayloadDef/RateCmdMsgPayload.h>
#include <architecture/msgPayloadDef/VehicleConfigMsgPayload.h>

#include <Eigen/Core>

/*! @brief The configuration structure for the rateServoFullNonlinear module.  */
class RateServoFullNonlinearAlgorithm {
   public:
    void reset(VehicleConfigMsgPayload vehConfigMsg, RWArrayConfigMsgPayload rwConfigMsg, bool rwIsLinked);
    CmdTorqueBodyMsgPayload update(uint64_t callTime,
                                   AttGuidMsgPayload guidCmd,
                                   RateCmdMsgPayload rateCmd,
                                   RWSpeedMsgPayload wheelSpeeds,
                                   RWAvailabilityMsgPayload wheelsAvailability);

    void setP(const double gain);
    double getP() const;
    void setKi(const double gain);
    double getKi() const;
    void setIntegralLimit(const double limit);
    double getIntegralLimit() const;
    void setKnownTorquePntB_B(const Eigen::Vector3d& knownTorquePntB_B);
    Eigen::Vector3d getKnownTorquePntB_B() const;

   private:
    double P{};              //!< [N*m*s]   Rate error feedback gain applied
    double Ki{};             //!< [N*m]     Integration feedback error on rate error
    double integralLimit{};  //!< [N*m]     Integration limit to avoid wind-up issue
    Eigen::Vector3d knownTorquePntB_B{
        Eigen::Vector3d::Zero()};  //!< [N*m]     known external torque in body frame vector components
    uint64_t priorTime{};          //!< [ns]      Last time the attitude control is called
    Eigen::Vector3d z{};           //!< [rad]     integral state of delta_omega
    Eigen::Matrix3d ISCPntB_B{};   //!< [kg m^2] Spacecraft Inertia
    RWArrayConfigMsgPayload
        rwConfigParams{};  //!< [-] struct to store message containing RW config parameters in body B frame
};

#endif
