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
    MRP_STEERING Module

 */

#include "fswAlgorithms/attControl/mrpSteering/mrpSteering.h"
#include "architecture/utilities/avsEigenSupport.h"
#include "architecture/utilities/rigidBodyKinematics.hpp"
#include <math.h>
#include <stdexcept>

/*! This method performs a complete reset of the module.  Local module variables that retain
 time varying states between function calls are reset to their default values.
 @return void
 @param callTime The clock time at which the function was called (nanoseconds)
*/
void MrpSteering::reset(uint64_t callTime)
{
    // check for required input message
    if (!this->guidInMsg.isLinked()) {
        throw std::invalid_argument("mrpSteering.guidInMsg wasn't connected.");
    }

    return;
}

/*! This method takes the attitude and rate errors relative to the Reference frame, as well as
    the reference frame angular rates and acceleration
 @return void
 @param callTime The clock time at which the function was called (nanoseconds)
 */
void MrpSteering::updateState(uint64_t callTime)
{
    AttGuidMsgPayload guidCmd;              /* Guidance Message */
    RateCmdMsgPayload outMsg = {};          /* copy of output message */

    /*! - Read the dynamic input messages */
    guidCmd = this->guidInMsg();

    Eigen::Vector3d sigma_BR = Eigen::Map<const Eigen::Vector3d>(guidCmd.sigma_BR);
    Eigen::Vector3d omega_ast{};
    Eigen::Vector3d omega_ast_p{Eigen::Vector3d::Zero()};

    double  sigma_i;        /* ith component of sigma_B/R */
    double  value;
    int     i;

    /* Equation (18): Determine the desired steering rates  */
    for (i=0;i<3;i++) {
        sigma_i  = sigma_BR[i];
        value = atan(M_PI_2/this->omega_max*(this->K1*sigma_i
                       + this->K3*sigma_i*sigma_i*sigma_i))/M_PI_2*this->omega_max;
        omega_ast[i] = -value;
    }
    if (!this->ignoreOuterLoopFeedforward) {
        /* Equation (21): Determine the body frame derivative of the steering rates */
        Eigen::Matrix3d B = bmatMrp(sigma_BR);
        Eigen::Vector3d sigmaDot_BR = 0.25 * B * omega_ast;

        for (i=0;i<3;i++) {
            sigma_i  = sigma_BR[i];
            value = (3*this->K3*sigma_i*sigma_i + this->K1)/
                                (pow(M_PI_2/this->omega_max*(this->K1*sigma_i + this->K3*sigma_i*sigma_i*sigma_i),2) + 1);
            omega_ast_p[i] = - value*sigmaDot_BR[i];
        }
    }

    eigenVector3d2CArray(omega_ast, outMsg.omega_BastR_B);
    eigenVector3d2CArray(omega_ast_p, outMsg.omegap_BastR_B);

    /*! - Store the output message and pass it to the message bus */
    this->rateCmdOutMsg.write(&outMsg, moduleID, callTime);

    return;
}
