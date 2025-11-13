#include "mrpPD.h"
#include <architecture/utilities/eigenSupport.h>

/*! Reset method for the BSK module adapter interface. This method also calls the algorithm reset method.
 @return void
 @param callTime [ns] Time the method is called
*/
void MrpPD::reset(uint64_t callTime) {
    if (!this->guidInMsg.isLinked()) {
        throw std::invalid_argument("mrpPD.guidInMsg wasn't connected.");
    }
    if (!this->vehConfigInMsg.isLinked()) {
        throw std::invalid_argument("mrpPD.vehConfigInMsg wasn't connected.");
    }

    if (this->vehConfigInMsg.isWritten()) {
        this->algorithm.setSpacecraftInertia(this->vehConfigInMsg());
    }
}

/*! Update method for the BSK module adapter interface. This method also calls the algorithm update method.
 @return void
 @param callTime [ns] Time the method is called
*/
void MrpPD::updateState(uint64_t callTime) {
    auto localGuidInMsg = AttGuidMsgPayload();
    if (this->guidInMsg.isWritten()) {
        localGuidInMsg = this->guidInMsg();
    }

    // Call the algorithm update method
    CmdTorqueBodyMsgPayload torqueCmdMsgPayload = this->algorithm.update(callTime, localGuidInMsg);

    this->cmdTorqueOutMsg.write(&torqueCmdMsgPayload, moduleID, callTime);
}

/*! Setter method for the derivative gain P.
 @return void
 @param P [N*m*s] Rate error feedback gain applied
*/
void MrpPD::setDerivativeGainP(double P) { this->algorithm.setDerivativeGainP(P); }

/*! Getter method for the derivative gain P.
 @return double
*/
double MrpPD::getDerivativeGainP() const { return this->algorithm.getDerivativeGainP(); }

/*! Setter method for the known external torque about point B.
 @return void
 @param knownTorquePntB_B [N*m] Known external torque expressed in body frame components
*/
void MrpPD::setKnownTorquePntB_B(Eigen::Vector3d& knownTorquePntB_B) {
    this->algorithm.setKnownTorquePntB_B(knownTorquePntB_B);
}

/*! Getter method for the known torque about point B.
 @return const Eigen::Vector3d&
*/
const Eigen::Vector3d& MrpPD::getKnownTorquePntB_B() const { return this->algorithm.getKnownTorquePntB_B(); }

/*! Setter method for the proportional gain K.
 @return void
 @param K [rad/s] Proportional gain applied to MRP errors
*/
void MrpPD::setProportionalGainK(double K) { this->algorithm.setProportionalGainK(K); }

/*! Getter method for the proportional gain K.
 @return double
*/
double MrpPD::getProportionalGainK() const { return this->algorithm.getProportionalGainK(); }
