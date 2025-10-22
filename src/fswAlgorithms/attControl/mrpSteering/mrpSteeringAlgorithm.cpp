/*
 ISC License

 Copyright (c) 2025, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

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

#include "mrpSteeringAlgorithm.h"
#include <architecture/utilities/eigenSupport.h>
#include <architecture/utilities/rigidBodyKinematics.hpp>
#include <math.h>
#include <stdint.h>
#include <Eigen/Core>
#include <stdexcept>

/*! This method takes the attitude and rate errors relative to the Reference frame, as well as
    the reference frame angular rates and acceleration
 @return RateCmdMsgPayload
 @param guidInMsg attitude guidance input message
 */
RateCmdMsgPayload MrpSteeringAlgorithm::update(AttGuidMsgPayload& guidInMsg) const {
    Eigen::Vector3d sigma_BR = Eigen::Map<const Eigen::Vector3d>(guidInMsg.sigma_BR);

    Eigen::Vector3d omega_ast{};
    Eigen::Vector3d omega_ast_p{Eigen::Vector3d::Zero()};

    for (uint32_t i = 0; i < 3; ++i) {
        double sigma_i = sigma_BR[i];
        double f_i =
            atan(M_PI_2 / this->omegaMax * (this->K1 * sigma_i + this->K3 * pow(sigma_i, 3))) / M_PI_2 * this->omegaMax;
        omega_ast[i] = -f_i;
    }
    if (!this->ignoreOuterLoopFeedforward) {
        Eigen::Matrix3d B = bmatMrp(sigma_BR);
        Eigen::Vector3d sigmaDot_BR = 0.25 * B * omega_ast;

        for (uint32_t i = 0; i < 3; ++i) {
            double sigma_i = sigma_BR[i];
            double f_i = (3 * this->K3 * pow(sigma_i, 2) + this->K1) /
                         (pow(M_PI_2 / this->omegaMax * (this->K1 * sigma_i + this->K3 * pow(sigma_i, 3)), 2) + 1);
            omega_ast_p[i] = -f_i * sigmaDot_BR[i];
        }
    }
    RateCmdMsgPayload outMsg{};

    eigenVectorToCArray(omega_ast, outMsg.omega_BastR_B);
    eigenVectorToCArray(omega_ast_p, outMsg.omegap_BastR_B);

    return outMsg;
}

/*! Set the linear feedback gain K1
 @return void
 @param gain [-] linear feedback gain K1
*/
void MrpSteeringAlgorithm::setK1(const double gain) {
    if (gain < 0.0) {
        throw std::invalid_argument("mrpSteering feedback gain K1 must not be negative");
    }
    this->K1 = gain;
}

/*! Get the linear feedback gain K1
 @return double
*/
double MrpSteeringAlgorithm::getK1() const { return this->K1; }

/*! Set the cubic feedback gain K3
 @return void
 @param gain [-] cubic feedback gain K3
*/
void MrpSteeringAlgorithm::setK3(const double gain) {
    if (gain < 0.0) {
        throw std::invalid_argument("mrpSteering feedback gain K3 must not be negative");
    }
    this->K3 = gain;
}

/*! Get the cubic feedback gain K3
 @return double
*/
double MrpSteeringAlgorithm::getK3() const { return this->K3; }

/*! Set the maximum rate command of steering control
 @return void
 @param omega [-] maximum rate command of steering control
*/
void MrpSteeringAlgorithm::setOmegaMax(const double omega) {
    if (omega <= 0.0) {
        throw std::invalid_argument("mrpSteering maximum rate omegaMax must be positive");
    }
    this->omegaMax = omega;
}

/*! Get the maximum rate command of steering control
 @return double
*/
double MrpSteeringAlgorithm::getOmegaMax() const { return this->omegaMax; }

/*! Set whether the outer loop feed-forward is ignored
 @return void
 @param ignore boolean whether the outer loop feed-forward should be ignored
*/
void MrpSteeringAlgorithm::setIgnoreFeedforward(const bool ignore) { this->ignoreOuterLoopFeedforward = ignore; }

/*! Get whether the outer loop feed-forward is ignored
 @return bool
*/
bool MrpSteeringAlgorithm::getIgnoreFeedforward() const { return this->ignoreOuterLoopFeedforward; }
