#include "mrpRotation.h"
#include <stdexcept>

/*! @brief This resets the module to original states.
 @return void
 @param callTime The clock time at which the function was called (nanoseconds)
 */
void MrpRotation::reset(uint64_t callTime) {
    // check if the required input messages are included
    if (!this->attRefInMsg.isLinked()) {
        throw std::invalid_argument("mrpRotation.attRefInMsg wasn't connected.");
    }

    // Check if optional input messages are linked
    if (this->desiredAttInMsg.isLinked()) {
        this->algorithm.enableDynamicReference();
    }

    this->algorithm.reset();
}

/*! @brief This method takes the input attitude reference frame, and superimposes the dynamics MRP
 scanning motion on top of this.
 @return void
 @param callTime The clock time at which the function was called (nanoseconds)
 */
void MrpRotation::updateState(uint64_t callTime) {
    AttRefMsgPayload inputRef = this->attRefInMsg();
    AttStateMsgPayload attStates{};

    if (this->algorithm.isDynamicReferenceEnabled()) {
        attStates = this->desiredAttInMsg();
    }

    AttRefMsgPayload attRefOut = this->algorithm.update(callTime, inputRef, attStates);

    /*! - write attitude guidance reference output */
    this->attRefOutMsg.write(&attRefOut, this->moduleID, callTime);
}

/*! Setter method for the current MRP attitude coordinate set with respect to the input reference
 @return void
 @param sigma [-] current MRP attitude coordinate set with respect to the input reference
*/
void MrpRotation::setSigmaRR0(const Eigen::Vector3d& sigma) { this->algorithm.setSigmaRR0(sigma); }

/*! Getter method for the current MRP attitude coordinate set with respect to the input reference
 @return const Eigen::Vector3d
*/
const Eigen::Vector3d MrpRotation::getSigmaRR0() const { return this->algorithm.getSigmaRR0(); }

/*! Setter method for the angular velocity vector relative to input reference
 @return void
 @param omega [rad/s] angular velocity vector relative to input reference
*/
void MrpRotation::setOmegaRR0(const Eigen::Vector3d& omega) { this->algorithm.setOmegaRR0(omega); }

/*! Getter method for the angular velocity vector relative to input reference
 @return const Eigen::Vector3d
*/
const Eigen::Vector3d MrpRotation::getOmegaRR0() const { return this->algorithm.getOmegaRR0(); }
