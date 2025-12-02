#include "mrpPDAlgorithm.h"
#include <architecture/utilities/eigenSupport.h>
#include <cmath>

/*! Update method for mrpPD control algorithm. This method takes the attitude and rate errors relative to the
 reference frame, as well as the reference frame angular rates and acceleration, and computes the required control
 torque Lr.
 @return void
 @param callTime [ns] Time the method is called
*/
CmdTorqueBodyMsgPayload MrpPDAlgorithm::update(uint64_t callTime, AttGuidMsgPayload guidInMsg) {
    // Compute hub inertial angular velocity in B-frame components
    Eigen::Vector3d omega_BR_B = cArrayToEigenVector(guidInMsg.omega_BR_B);
    Eigen::Vector3d omega_RN_B = cArrayToEigenVector(guidInMsg.omega_RN_B);
    Eigen::Vector3d omega_BN_B = omega_BR_B + omega_RN_B;

    Eigen::Vector3d sigma_BR = cArrayToEigenVector(guidInMsg.sigma_BR);
    Eigen::Vector3d domega_RN_B = cArrayToEigenVector(guidInMsg.domega_RN_B);

    // Compute required attitude control torque vector
    Eigen::Vector3d Lr = -this->K * sigma_BR - this->P * omega_BR_B + omega_RN_B.cross(this->ISCPntB_B * omega_BN_B) +
                         this->ISCPntB_B * (domega_RN_B - omega_BN_B.cross(omega_RN_B)) -
                         this->knownTorquePntB_B;  // [Nm]

    // Create the output message
    auto torqueCmdMsgPayload = CmdTorqueBodyMsgPayload();
    eigenVectorToCArray(Lr, torqueCmdMsgPayload.torqueRequestBody);

    return torqueCmdMsgPayload;
}

/*! This method sets the spacecraft inertia according to the vehicle configuration input message
 @return void
 @param vehicleConfigIn Vehicle config input
*/
void MrpPDAlgorithm::setSpacecraftInertia(VehicleConfigMsgPayload vehicleConfigIn) {
    this->ISCPntB_B = cArrayToEigenMatrix3(vehicleConfigIn.ISCPntB_B);
}

/*! Setter method for the derivative gain P.
 @return void
 @param P [N*m*s] Rate error feedback gain applied
*/
void MrpPDAlgorithm::setDerivativeGainP(double P) {
    if (P < 0.0) {
        throw std::invalid_argument("Feedback gain P must not be negative");
    }
    this->P = P;
}

/*! Getter method for the derivative gain P.
 @return double
*/
double MrpPDAlgorithm::getDerivativeGainP() const { return this->P; }

/*! Setter method for the known external torque about point B.
 @return void
 @param knownTorquePntB_B [N*m] Known external torque expressed in body frame components
*/
void MrpPDAlgorithm::setKnownTorquePntB_B(Eigen::Vector3d& knownTorquePntB_B) {
    this->knownTorquePntB_B = knownTorquePntB_B;
}

/*! Getter method for the known torque about point B.
 @return const Eigen::Vector3d&
*/
const Eigen::Vector3d& MrpPDAlgorithm::getKnownTorquePntB_B() const { return this->knownTorquePntB_B; }

/*! Setter method for the proportional gain K.
 @return void
 @param K [rad/s] Proportional gain applied to MRP errors
*/
void MrpPDAlgorithm::setProportionalGainK(double K) {
    if (K < 0.0) {
        throw std::invalid_argument("Feedback gain K must not be negative");
    }
    this->K = K;
}

/*! Getter method for the proportional gain K.
 @return double
*/
double MrpPDAlgorithm::getProportionalGainK() const { return this->K; }
