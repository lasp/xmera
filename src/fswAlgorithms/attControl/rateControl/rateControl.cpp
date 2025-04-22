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

#include "rateControl.h"

#include "architecture/utilities/avsEigenSupport.h"
#include "architecture/utilities/linearAlgebra.h"

/*! This method performs a complete reset of the module.  Local module variables that retain
 time varying states between function calls are reset to their default values.
 @return void
 @param callTime [ns] Time the method is called
*/
void RateControl::reset(uint64_t callTime) {
    if (!this->guidInMsg.isLinked()) {
        _bskLog(this->bskLogger, BSK_ERROR, "rateControl.guidInMsg wasn't connected.");
    }
    if (!this->vehConfigInMsg.isLinked()) {
        _bskLog(this->bskLogger, BSK_ERROR, "rateControl.vehConfigInMsg wasn't connected.");
    }

    // Read the VehicleConfigMsgPayload input message
    if (this->vehConfigInMsg.isWritten()) {
        VehicleConfigMsgPayload vcInMsg = this->vehConfigInMsg();
        this->ISCPntB_B = cArray2EigenMatrixXd(vcInMsg.ISCPntB_B, 3, 3);
    }
}

/*! This method takes the attitude and rate errors relative to the reference frame, as well as
the reference frame angular rates and acceleration, and computes the required control torque Lr.
 @return void
 @param callTime [ns] Time the method is called
*/
void RateControl::updateState(uint64_t callTime) {
    // Read the guidance input message
    auto guidanceMsgPayload = AttGuidMsgPayload();
    if (this->guidInMsg.isWritten()) {
        guidanceMsgPayload = this->guidInMsg();
    }

    // Compute required attitude control torque vector
    Eigen::Vector3d omega_BR_B = cArray2EigenVector3d(guidanceMsgPayload.omega_BR_B);
    Eigen::Vector3d omega_RN_B = cArray2EigenVector3d(guidanceMsgPayload.omega_RN_B);
    Eigen::Vector3d omega_BN_B = omega_BR_B + omega_RN_B;
    Eigen::Vector3d domega_RN_B = cArray2EigenVector3d(guidanceMsgPayload.domega_RN_B);
    Eigen::Vector3d Lr = -this->P * omega_BR_B + omega_RN_B.cross(this->ISCPntB_B * omega_BN_B) +
                         this->ISCPntB_B * (domega_RN_B - omega_BN_B.cross(omega_RN_B)) -
                         this->knownTorquePntB_B;  // [Nm]

    // Create and write the output message
    auto torqueCmdMsgPayload = CmdTorqueBodyMsgPayload();
    eigenVector3d2CArray(Lr, torqueCmdMsgPayload.torqueRequestBody);
    this->cmdTorqueOutMsg.write(&torqueCmdMsgPayload, moduleID, callTime);
}

/*! Getter method for the derivative gain P.
 @return const double
*/
const double RateControl::getDerivativeGainP() const { return this->P; }

/*! Getter method for the known torque about point B.
 @return const Eigen::Vector3d
*/
const Eigen::Vector3d &RateControl::getKnownTorquePntB_B() const { return this->knownTorquePntB_B; }

/*! Setter method for the derivative gain P.
 @return void
 @param P [N*m*s] Rate error feedback gain applied
*/
void RateControl::setDerivativeGainP(const double P) { this->P = P; }

/*! Setter method for the known external torque about point B.
 @return void
 @param knownTorquePntB_B [N*m] Known external torque expressed in body frame components
*/
void RateControl::setKnownTorquePntB_B(const Eigen::Vector3d &knownTorquePntB_B) {
    this->knownTorquePntB_B = knownTorquePntB_B;
}
