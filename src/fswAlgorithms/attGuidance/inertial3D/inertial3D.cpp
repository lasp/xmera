/*
 ISC License

 Copyright (c) 2025, Laboratory for Atmospheric Space Physics, University of Colorado at Boulder

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

#include "inertial3D.h"

/*! This method creates a fixed attitude reference message.  The desired orientation is
    defined within the module.
 @return void
 @param callTime The clock time at which the function was called (nanoseconds)
 */
void Inertial3D::updateState(uint64_t callTime) {
    AttRefMsgPayload attRefOut = algorithm.update();

    this->attRefOutMsg.write(&attRefOut, this->moduleID, callTime);
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
