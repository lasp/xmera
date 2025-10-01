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
    AttGuidMsgPayload guidCmd = this->guidInMsg();
    RateCmdMsgPayload outMsg{};

    Eigen::Vector3d sigma_BR = Eigen::Map<const Eigen::Vector3d>(guidCmd.sigma_BR);

    Eigen::Vector3d omega_ast{};
    Eigen::Vector3d omega_ast_p{Eigen::Vector3d::Zero()};

    for (uint32_t i=0; i<3; ++i) {
        double sigma_i = sigma_BR[i];
        double f_i = atan(M_PI_2/this->omegaMax*(this->K1*sigma_i + this->K3*pow(sigma_i,3)))/M_PI_2*this->omegaMax;
        omega_ast[i] = - f_i;
    }
    if (!this->ignoreOuterLoopFeedforward) {
        Eigen::Matrix3d B = bmatMrp(sigma_BR);
        Eigen::Vector3d sigmaDot_BR = 0.25 * B * omega_ast;

        for (uint32_t i=0; i<3; ++i) {
            double sigma_i = sigma_BR[i];
            double f_i = (3*this->K3*pow(sigma_i,2) + this->K1)/
                         (pow(M_PI_2/this->omegaMax*(this->K1*sigma_i + this->K3*pow(sigma_i,3)),2) + 1);
            omega_ast_p[i] = - f_i * sigmaDot_BR[i];
        }
    }

    eigenVector3d2CArray(omega_ast, outMsg.omega_BastR_B);
    eigenVector3d2CArray(omega_ast_p, outMsg.omegap_BastR_B);

    this->rateCmdOutMsg.write(&outMsg, moduleID, callTime);

    return;
}

/*! Set the linear feedback gain K1
 @return void
 @param gain [-] linear feedback gain K1
*/
void MrpSteering::setK1(const double gain) {
    if (gain < 0.0) {
        throw std::invalid_argument("mrpSteering feedback gain K1 must not be negative");
    }
    this->K1 = gain;
}

/*! Get the linear feedback gain K1
 @return double
*/
double MrpSteering::getK1() const { return this->K1; }

/*! Set the cubic feedback gain K3
 @return void
 @param gain [-] cubic feedback gain K3
*/
void MrpSteering::setK3(const double gain) {
    if (gain < 0.0) {
        throw std::invalid_argument("mrpSteering feedback gain K3 must not be negative");
    }
    this->K3 = gain;
}

/*! Get the cubic feedback gain K3
 @return double
*/
double MrpSteering::getK3() const { return this->K3; }

/*! Set the maximum rate command of steering control
 @return void
 @param omega [-] maximum rate command of steering control
*/
void MrpSteering::setOmegaMax(const double omega) {
    if (omega <= 0.0) {
        throw std::invalid_argument("mrpSteering maximum rate omegaMax must be positive");
    }
    this->omegaMax = omega;
}

/*! Get the maximum rate command of steering control
 @return double
*/
double MrpSteering::getOmegaMax() const { return this->omegaMax; }

/*! Set whether the outer loop feed-forward is ignored
 @return void
 @param ignore boolean whether the outer loop feed-forward should be ignored
*/
void MrpSteering::setIgnoreFeedforward(const bool ignore) { this->ignoreOuterLoopFeedforward = ignore; }

/*! Get whether the outer loop feed-forward is ignored
 @return bool
*/
bool MrpSteering::getIgnoreFeedforward() const { return this->ignoreOuterLoopFeedforward; }
