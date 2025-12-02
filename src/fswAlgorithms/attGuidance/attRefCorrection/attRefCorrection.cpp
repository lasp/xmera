#include <architecture/utilities/eigenSupport.h>
#include <architecture/utilities/rigidBodyKinematics.hpp>
#include "attRefCorrection.h"

#include <stdexcept>

/*! This method performs a complete reset of the module.  Local module variables that retain
    time varying states between function calls are reset to their default values.
    Check if required input messages are connected.
 @return void
 @param callTime [ns] time the method is called
*/
void AttRefCorrection::reset(uint64_t callTime) {
    // check if the required message has not been connected
    if (!this->attRefInMsg.isLinked()) {
        throw std::invalid_argument("attRefCorrection.attRefInMsg wasn't connected.");
    }
}

/*! Corrects the reference attitude message by a fixed rotation
 @return void
 @param callTime The clock time at which the function was called (nanoseconds)
*/
void AttRefCorrection::updateState(uint64_t const callTime) {
    // read in the input messages
    AttRefMsgPayload attRefMsgBuffer = this->attRefInMsg();
    Eigen::Vector3d sigma_RN_local = cArrayToEigenVector3(attRefMsgBuffer.sigma_RN);

    // compute corrected reference orientation
    sigma_RN_local = addMrp(sigma_RN_local, this->sigma_RR0);

    // write to the output messages
    eigenVectorToCArray(sigma_RN_local, attRefMsgBuffer.sigma_RN);
    this->attRefOutMsg.write(&attRefMsgBuffer, this->moduleID, callTime);
}

/*! Setter method for the current MRP attitude coordinate set with respect to the input reference
 @return void
 @param sigmaRR0 [-] current MRP attitude coordinate set with respect to the input reference
*/
void AttRefCorrection::setSigmaRR0(const Eigen::Vector3d& sigma) { this->sigma_RR0 = sigma; }

/*! Getter method for the current MRP attitude coordinate set with respect to the input reference
 @return const Eigen::Vector3d
*/
const Eigen::Vector3d AttRefCorrection::getSigmaRR0() const { return this->sigma_RR0; }
