#include "attTrackingErrorAlgorithm.h"

#include <architecture/utilities/eigenSupport.h>
#include <architecture/utilities/linearAlgebra.h>
#include <architecture/utilities/rigidBodyKinematics.hpp>

/*! This method performs a complete reset of the module.  Local module variables that retain
 time varying states between function calls are reset to their default values.
 @return void
 @param callTime The clock time at which the function was called (nanoseconds)
 */
void AttTrackingErrorAlgorithm::reset(uint64_t callTime) {
    // Reset the algorithm
}

/*! This method maps the input thruster command forces into thruster on times using a remainder tracking logic.
 @return void
 @param callTime The clock time at which the function was called (nanoseconds)
 */
AttGuidMsgPayload AttTrackingErrorAlgorithm::update(uint64_t callTime,
                                                    AttRefMsgPayload& attRefInMsg,
                                                    NavAttMsgPayload& attNavInMsg) {
    AttGuidMsgPayload attGuidOut;

    // Compute MRP from the original reference frame R0 to the corrected reference frame R
    Eigen::Vector3d sigma_RR0 = -1 * this->sigma_R0R;

    // Compute MRP from inertial to updated reference frame sigma_RN
    Eigen::Vector3d sigma_RN = cArrayToEigenVector(attRefInMsg.sigma_RN);
    sigma_RN = addMrp(sigma_RN, sigma_RR0);

    // Compute attitude error sigma_BR
    Eigen::Vector3d sigma_BN = cArrayToEigenVector(attNavInMsg.sigma_BN);
    Eigen::Vector3d sigma_BR = subMrp(sigma_BN, sigma_RN);

    // Compute angular velocity reference body frame components omega_RN_B
    Eigen::Matrix3d dcm_BN = mrpToDcm(sigma_BN);
    Eigen::Vector3d omega_RN_N = cArrayToEigenVector(attRefInMsg.omega_RN_N);
    Eigen::Vector3d omega_RN_B = dcm_BN * omega_RN_N;

    // Compute angular velocity error omega_BR
    Eigen::Vector3d omega_BN_B = cArrayToEigenVector(attNavInMsg.omega_BN_B);
    Eigen::Vector3d omega_BR_B = omega_BN_B - omega_RN_B;

    // Compute reference angular velocity rate in body frame components domega_RN_B
    Eigen::Vector3d domega_RN_N = cArrayToEigenVector(attRefInMsg.domega_RN_N);
    Eigen::Vector3d domega_RN_B = dcm_BN * domega_RN_N;

    // Write attitude guidance output message
    eigenVectorToCArray(omega_RN_B, attGuidOut.omega_RN_B);
    eigenVectorToCArray(omega_BR_B, attGuidOut.omega_BR_B);
    eigenVectorToCArray(sigma_BR, attGuidOut.sigma_BR);
    eigenVectorToCArray(domega_RN_B, attGuidOut.domega_RN_B);

    return attGuidOut;
}

/*! Setter method for sigma_R0R.
 @return void
 @param sigma_R0R
*/
void AttTrackingErrorAlgorithm::setSigma_R0R(const Eigen::Vector3d& sigma_R0R) { this->sigma_R0R = sigma_R0R; }

/*! Getter method for sigma_R0R.
 @return const Eigen::Vector3d
*/
const Eigen::Vector3d& AttTrackingErrorAlgorithm::getSigma_R0R() const { return this->sigma_R0R; }
