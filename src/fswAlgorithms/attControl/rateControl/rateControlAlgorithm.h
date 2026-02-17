// SPDX-License-Identifier: ISC
// Copyright (c) 2025, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

#ifndef _RATE_CONTROL_ALGORITHM
#define _RATE_CONTROL_ALGORITHM

#include <stdexcept>

#include <Eigen/Dense>

#include <architecture/msgPayloadDef/AttGuidMsgPayload.h>
#include <architecture/msgPayloadDef/CmdTorqueBodyMsgPayload.h>
#include <architecture/msgPayloadDef/VehicleConfigMsgPayload.h>

class RateControlAlgorithm {
   public:
    CmdTorqueBodyMsgPayload update(AttGuidMsgPayload attGuidIn);
    void setSpacecraftInertia(VehicleConfigMsgPayload vehicleConfigIn);
    void setDerivativeGainP(double P);
    double getDerivativeGainP() const;
    void setKnownTorquePntB_B(const Eigen::Vector3d& knownTorquePntB_B);
    const Eigen::Vector3d& getKnownTorquePntB_B() const;

   private:
    double P{};  //!< [N*m*s] Rate error feedback gain applied
    Eigen::Vector3d knownTorquePntB_B{
        Eigen::Vector3d::Zero()};  //!< [N*m] Known external torque expressed in body frame components
    Eigen::Matrix3d ISCPntB_B{
        Eigen::Matrix3d::Identity()};  //!< [kg*m^2] Spacecraft inertia about point B expressed in body frame components
};

#endif
