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
/*
    Inertial 3D Spin Module

 */

/* modify the path to reflect the new module names */
#include "fswAlgorithms/attGuidance/inertial3D/inertial3D.h"
#include "architecture/utilities/avsEigenSupport.h"

/*! This method creates a fixed attitude reference message.  The desired orientation is
    defined within the module.
 @return AttRefMsgPayload
 */
AttRefMsgPayload Inertial3DAlgorithm::update()
{
    AttRefMsgPayload attRefOut{};
    eigenVector3d2CArray(this->sigma_R0N, attRefOut.sigma_RN);

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
