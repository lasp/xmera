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

#include "sunlineEphem.h"
#include "architecture/utilities/rigidBodyKinematics.hpp"

/*! Updates the sun heading based on ephemeris data. Returns the heading as a unit vector in the body frame.
 @return void
 @param callTime The clock time at which the function was called (nanoseconds)
 */
void SunlineEphem::updateState(uint64_t callTime) {
    this->readMessages();
    Eigen::Vector3d r_SB_B_hat = this->algorithm();
    this->writeMessages(callTime, r_SB_B_hat);
}

void SunlineEphem::readMessages() {
    // check if the required input messages are included
    if (!this->sunPositionInMsg.isLinked()) {
        this->bskLogger.bskLog(BSK_ERROR, "Error: sunlineEphem.sunPositionInMsg wasn't connected.");
    }
    if (!this->scPositionInMsg.isLinked()) {
        this->bskLogger.bskLog(BSK_ERROR, "Error: sunlineEphem.scPositionInMsg wasn't connected.");
    }
    if (!this->scAttitudeInMsg.isLinked()) {
        this->bskLogger.bskLog(BSK_ERROR, "Error: sunlineEphem.scAttitudeInMsg wasn't connected.");
    }
}

Eigen::Vector3d SunlineEphem::algorithm() {
    /*! - Calculate Sunline Heading from Ephemeris Data*/
    const Eigen::Vector3d rSun(this->sunPositionInMsg().r_BdyZero_N[0],
                               this->sunPositionInMsg().r_BdyZero_N[1],
                               this->sunPositionInMsg().r_BdyZero_N[2]);
    const Eigen::Vector3d rSc(
        this->scPositionInMsg().r_BN_N[0], this->scPositionInMsg().r_BN_N[1], this->scPositionInMsg().r_BN_N[2]);
    // Difference in inertial frame
    const Eigen::Vector3d r_SB_N = rSun - rSc;

    // Prepare the unit-length inertial vector (defaults to zero)
    Eigen::Vector3d r_SB_N_hat = Eigen::Vector3d::Zero();
    if (r_SB_N.norm() > std::numeric_limits<double>::epsilon()) {
        r_SB_N_hat = r_SB_N;
        r_SB_N_hat.normalize();  // in-place unit-length
    }

    // Build DCM from spacecraft attitude
    const Eigen::Vector3d sigma_BN(
        this->scAttitudeInMsg().sigma_BN[0], this->scAttitudeInMsg().sigma_BN[1], this->scAttitudeInMsg().sigma_BN[2]);
    const Eigen::Matrix3d dcm_BN = mrpToDcm(sigma_BN);

    // Rotate into body frame
    Eigen::Vector3d r_SB_B_hat = dcm_BN * r_SB_N_hat;

    // Ensure unit length (or zero)
    if (r_SB_B_hat.norm() > std::numeric_limits<double>::epsilon()) {
        r_SB_B_hat.normalize();  // in-place unit-length
    } else {
        r_SB_B_hat.setZero();  // explicit zero
    }

    return r_SB_B_hat;
}

void SunlineEphem::writeMessages(uint64_t callTime, Eigen::Vector3d r_SB_B_hat) {
    /*! - store the output message*/
    NavAttMsgPayload outputSunline = {}; /* [-] Output sunline estimate data */
    for (int i = 0; i < 3; i++) {
        outputSunline.vehSunPntBdy[i] = r_SB_B_hat[i];
    }
    this->navStateOutMsg.write(&outputSunline, this->moduleID, callTime);
}
