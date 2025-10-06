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

#include "architecture/utilities/eigenSupport.h"

/*! This method takes the attitude and rate errors relative to the reference frame, as well as
the reference frame angular rates and acceleration, and computes the required control torque Lr.
 @return torqueCmdOut
 @param attGuidIn Attitude guidance input
*/
CmdTorqueBodyMsgPayload RateControlAlgorithm::update(AttGuidMsgPayload attGuidIn) {
    CmdTorqueBodyMsgPayload torqueCmdOut{};

    // Compute required attitude control torque vector
    Eigen::Vector3d omega_BR_B = cArrayAsEigenVector(attGuidIn.omega_BR_B);
    Eigen::Vector3d omega_RN_B = cArrayAsEigenVector(attGuidIn.omega_RN_B);
    Eigen::Vector3d omega_BN_B = omega_BR_B + omega_RN_B;
    Eigen::Vector3d domega_RN_B = cArrayAsEigenVector(attGuidIn.domega_RN_B);
    Eigen::Vector3d Lr = -this->P * omega_BR_B + omega_RN_B.cross(this->ISCPntB_B * omega_BN_B) +
                         this->ISCPntB_B * (domega_RN_B - omega_BN_B.cross(omega_RN_B)) -
                         this->knownTorquePntB_B;  // [Nm]

    eigenVectorToCArray(Lr, torqueCmdOut.torqueRequestBody);

    return torqueCmdOut;
}

/*! This method sets the spacecraft inertia according to the vehicle configuration input message
 @return void
 @param vehicleConfigIn Vehicle config input
*/
void RateControlAlgorithm::setSpacecraftInertia(VehicleConfigMsgPayload vehicleConfigIn) {
    this->ISCPntB_B = cArrayAsEigenMatrix3(vehicleConfigIn.ISCPntB_B);
}

/*! Setter method for the derivative gain P.
 @return void
 @param P [N*m*s] Rate error feedback gain applied
*/
void RateControlAlgorithm::setDerivativeGainP(const double P) {
    if (P < 0.0) {
        throw std::invalid_argument("Feedback gain P must not be negative");
    }
    this->P = P;
}

/*! Getter method for the derivative gain P.
 @return const double
*/
double RateControlAlgorithm::getDerivativeGainP() const { return this->P; }

/*! Setter method for the known external torque about point B.
 @return void
 @param knownTorquePntB_B [N*m] Known external torque expressed in body frame components
*/
void RateControlAlgorithm::setKnownTorquePntB_B(const Eigen::Vector3d& knownTorquePntB_B) {
    this->knownTorquePntB_B = knownTorquePntB_B;
}

/*! Getter method for the known torque about point B.
 @return const Eigen::Vector3d
*/
const Eigen::Vector3d& RateControlAlgorithm::getKnownTorquePntB_B() const { return this->knownTorquePntB_B; }
