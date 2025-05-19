/*
 ISC License

 Copyright (c) 2024, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

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

#include "mrpPD.h"
#include "architecture/utilities/avsEigenSupport.h"

/*! Reset method for the BSK module adapter interface. This method also calls the algorithm reset method.
 @return void
 @param callTime [ns] Time the method is called
*/
void MrpPD::reset(uint64_t callTime) {
    if (!this->guidInMsg.isLinked()) {
        _bskLog(this->bskLogger, BSK_ERROR, "mrpPD.guidInMsg wasn't connected.");
    }
    if (!this->vehConfigInMsg.isLinked()) {
        _bskLog(this->bskLogger, BSK_ERROR, "mrpPD.vehConfigInMsg wasn't connected.");
    }

    // Call the algorithm reset method
    VehicleConfigMsgPayload vcInMsg = this->vehConfigInMsg();
    this->algorithm.reset(callTime, vcInMsg);
}

/*! Update method for the BSK module adapter interface. This method also calls the algorithm update method.
 @return void
 @param callTime [ns] Time the method is called
*/
void MrpPD::updateState(uint64_t callTime) {
    AttGuidMsgPayload guidInMsg = this->guidInMsg();

    // Call the algorithm update method
    CmdTorqueBodyMsgPayload torqueCmdMsgPayload = this->algorithm.update(callTime, guidInMsg);

    this->cmdTorqueOutMsg.write(&torqueCmdMsgPayload, moduleID, callTime);
}

/*! Getter method for the derivative gain P.
 @return double
*/
double MrpPD::getDerivativeGainP() const { return this->algorithm.getDerivativeGainP(); }

/*! Getter method for the known torque about point B.
 @return const Eigen::Vector3d&
*/
const Eigen::Vector3d& MrpPD::getKnownTorquePntB_B() const { return this->algorithm.getKnownTorquePntB_B(); }

/*! Getter method for the proportional gain K.
 @return double
*/
double MrpPD::getProportionalGainK() const { return this->algorithm.getProportionalGainK(); }

/*! Setter method for the derivative gain P.
 @return void
 @param P [N*m*s] Rate error feedback gain applied
*/
void MrpPD::setDerivativeGainP(double P) { this->algorithm.setDerivativeGainP(P); }

/*! Setter method for the known external torque about point B.
 @return void
 @param knownTorquePntB_B [N*m] Known external torque expressed in body frame components
*/
void MrpPD::setKnownTorquePntB_B(Eigen::Vector3d& knownTorquePntB_B) {
    this->algorithm.setKnownTorquePntB_B(knownTorquePntB_B);
}

/*! Setter method for the proportional gain K.
 @return void
 @param K [rad/s] Proportional gain applied to MRP errors
*/
void MrpPD::setProportionalGainK(double K) { this->algorithm.setProportionalGainK(K); }
