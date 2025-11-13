#include <architecture/utilities/eigenSupport.h>
#include "inertial3D.h"

/*! This method creates a fixed attitude reference message.  The desired orientation is
    defined within the module.
 @return AttRefMsgPayload
 */
AttRefMsgPayload Inertial3DAlgorithm::update() {
    AttRefMsgPayload attRefOut{};
    eigenVectorToCArray(this->sigma_R0N, attRefOut.sigma_RN);

    return attRefOut;
}

/*! Setter method for the MRP from frame N to frame R.
 @return void
 @param sigma_RN [-] MRP from frame N to frame R
*/
void Inertial3DAlgorithm::setSigmaR0N(const Eigen::Vector3d& sigma_RN) { this->sigma_R0N = sigma_RN; }

/*! Getter method for the MRP from frame N to frame R.
 @return const Eigen::Vector3d
*/
const Eigen::Vector3d& Inertial3DAlgorithm::getSigmaR0N() const { return this->sigma_R0N; }
