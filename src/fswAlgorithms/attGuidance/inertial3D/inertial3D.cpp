// SPDX-License-Identifier: ISC
// Copyright (c) 2016, Autonomous Vehicle System Lab, University of Colorado at Boulder
// Copyright (c) 2025, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

#include "inertial3D.h"

/*! This method creates a fixed attitude reference message.  The desired orientation is
    defined within the module.
 @return void
 @param callTime The clock time at which the function was called (nanoseconds)
 */
void Inertial3D::updateState(uint64_t callTime) {
    AttRefMsgPayload attRefOut = algorithm.update();

    this->attRefOutMsg.write(attRefOut, this->moduleID, callTime);
}

/*! Setter method for the MRP from frame N to frame R.
 @return void
 @param sigma_RN [-] MRP from frame N to frame R
*/
void Inertial3D::setSigmaR0N(const Eigen::Vector3d& sigma_RN) { this->algorithm.setSigmaR0N(sigma_RN); }

/*! Getter method for the MRP from frame N to frame R.
 @return const Eigen::Vector3d
*/
const Eigen::Vector3d& Inertial3D::getSigmaR0N() const { return this->algorithm.getSigmaR0N(); }
