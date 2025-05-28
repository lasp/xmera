/*
 ISC License

 Copyright (c) 2025, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

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

#include "fswAlgorithms/attControl/mrpPD/mrpPDAlgorithm.h"
#include "architecture/utilities/avsEigenSupport.h"
#include <cassert>
#include <cmath>

/*! Reset method for the mrpPD control algorithm.
 @return void
 @param callTime [ns] Time the method is called
 @param vehConfigInMsg Vehicle configuration message
*/
void MrpPDAlgorithm::reset(uint64_t callTime, VehicleConfigMsgPayload vehConfigInMsg) {
    this->ISCPntB_B = cArray2EigenMatrix3d(vehConfigInMsg.ISCPntB_B);
}

/*! Update method for mrpPD control algorithm. This method takes the attitude and rate errors relative to the
 reference frame, as well as the reference frame angular rates and acceleration, and computes the required control
 torque Lr.
 @return void
 @param callTime [ns] Time the method is called
*/
CmdTorqueBodyMsgPayload MrpPDAlgorithm::update(uint64_t callTime, AttGuidMsgPayload guidInMsg) {
    // Compute hub inertial angular velocity in B-frame components
    Eigen::Vector3d omega_BR_B = cArray2EigenVector3d(guidInMsg.omega_BR_B);
    Eigen::Vector3d omega_RN_B = cArray2EigenVector3d(guidInMsg.omega_RN_B);
    Eigen::Vector3d omega_BN_B = omega_BR_B + omega_RN_B;

    Eigen::Vector3d sigma_BR = cArray2EigenVector3d(guidInMsg.sigma_BR);
    Eigen::Vector3d domega_RN_B = cArray2EigenVector3d(guidInMsg.domega_RN_B);

    // Compute required attitude control torque vector
    Eigen::Vector3d Lr = -this->K * sigma_BR - this->P * omega_BR_B + omega_RN_B.cross(this->ISCPntB_B * omega_BN_B) +
                         this->ISCPntB_B * (domega_RN_B - omega_BN_B.cross(omega_RN_B)) -
                         this->knownTorquePntB_B;  // [Nm]

    // Create the output message
    auto torqueCmdMsgPayload = CmdTorqueBodyMsgPayload();
    eigenVector3d2CArray(Lr, torqueCmdMsgPayload.torqueRequestBody);

    return torqueCmdMsgPayload;
}

/*! Getter method for the derivative gain P.
 @return double
*/
double MrpPDAlgorithm::getDerivativeGainP() const { return this->P; }

/*! Getter method for the known torque about point B.
 @return const Eigen::Vector3d&
*/
const Eigen::Vector3d& MrpPDAlgorithm::getKnownTorquePntB_B() const { return this->knownTorquePntB_B; }

/*! Getter method for the proportional gain K.
 @return double
*/
double MrpPDAlgorithm::getProportionalGainK() const { return this->K; }

/*! Setter method for the derivative gain P.
 @return void
 @param P [N*m*s] Rate error feedback gain applied
*/
void MrpPDAlgorithm::setDerivativeGainP(double P) {
    assert(P >= 0.0);
    this->P = std::abs(P);
}

/*! Setter method for the known external torque about point B.
 @return void
 @param knownTorquePntB_B [N*m] Known external torque expressed in body frame components
*/
void MrpPDAlgorithm::setKnownTorquePntB_B(Eigen::Vector3d& knownTorquePntB_B) {
    this->knownTorquePntB_B = knownTorquePntB_B;
}

/*! Setter method for the proportional gain K.
 @return void
 @param K [rad/s] Proportional gain applied to MRP errors
*/
void MrpPDAlgorithm::setProportionalGainK(double K) {
    assert(K >= 0.0);
    this->K = std::abs(K);
}
