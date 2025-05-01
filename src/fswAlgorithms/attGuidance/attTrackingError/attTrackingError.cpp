/*
 ISC License

 Copyright (c) 2016, Autonomous Vehicle Systems Lab, University of Colorado at Boulder

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

#include "fswAlgorithms/attGuidance/attTrackingError/attTrackingError.h"

#include "architecture/utilities/avsEigenSupport.h"
#include "architecture/utilities/linearAlgebra.h"
#include "architecture/utilities/rigidBodyKinematics.hpp"

/*! This method performs a complete reset of the module. Local module variables that retain time varying states between
 function calls are reset to their default values.
 @return void
 @param callTime The clock time at which the function was called (nanoseconds)
 */
void AttTrackingError::reset(uint64_t callTime) {
    // check if the required input messages are included
    if (!this->attRefInMsg.isLinked()) {
        this->bskLogger.bskLog(BSK_ERROR, "Error: attTrackingError.attRefInMsg wasn't connected.");
    }
    if (!this->attNavInMsg.isLinked()) {
        this->bskLogger.bskLog(BSK_ERROR, "Error: attTrackingError.attNavInMsg wasn't connected.");
    }

    this->algorithm.reset(callTime);
}

/*! The Update method performs reads the Navigation message (containing the spacecraft attitude information), and the
 Reference message (containing the desired attitude). It computes the attitude error and writes it in the Guidance
 message.
 @return void
 @param callTime The clock time at which the function was called (nanoseconds)
 */
void AttTrackingError::updateState(uint64_t callTime) {
    AttRefMsgPayload ref = this->attRefInMsg();
    NavAttMsgPayload nav = this->attNavInMsg();

    AttGuidMsgPayload attGuidOut = this->algorithm.update(callTime, ref, nav);
    this->attGuidOutMsg.write(&attGuidOut, this->moduleID, callTime);
}

/*! Setter method for sigma_R0R.
 @return void
 @param sigma_R0R
*/
void AttTrackingError::setSigma_R0R(const Eigen::Vector3d &sigma_R0R) { this->algorithm.setSigma_R0R(sigma_R0R); }

/*! Getter method for sigma_R0R.
 @return const Eigen::Vector3d
*/
const Eigen::Vector3d &AttTrackingError::getSigma_R0R() const { return this->algorithm.getSigma_R0R(); }
