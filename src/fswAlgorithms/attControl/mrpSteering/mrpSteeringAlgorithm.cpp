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

#include "fswAlgorithms/attControl/mrpSteering/mrpSteeringAlgorithm.h"
#include "architecture/utilities/avsEigenMRP.h"
#include "architecture/utilities/avsEigenSupport.h"
#include <cmath>

RateCmdMsgPayload MrpSteeringAlgorithm::update(uint64_t, AttGuidMsgPayload guidCmd) {
    RateCmdMsgPayload outMsg = {};

    Eigen::Vector3d sigma_BR = cArray2EigenVector3d(guidCmd.sigma_BR);
    Eigen::Vector3d omega_ast;
    Eigen::Vector3d omega_ast_p;

    for (int i = 0; i < 3; i++) {
        double sigma_i = sigma_BR[i];
        double value = std::atan(M_PI_2 / this->omega_max *
                                 (this->K1 * sigma_i +
                                  this->K3 * sigma_i * sigma_i * sigma_i)) /
                        M_PI_2 * this->omega_max;
        omega_ast[i] = -value;
    }

    omega_ast_p.setZero();

    if (!this->ignoreOuterLoopFeedforward) {
        Eigen::MRPd sigma(sigma_BR);
        Eigen::Vector3d sigma_p = 0.25 * sigma.Bmat() * omega_ast;
        for (int i = 0; i < 3; i++) {
            double sigma_i = sigma_BR[i];
            double value = (3.0 * this->K3 * sigma_i * sigma_i + this->K1) /
                           (std::pow(M_PI_2 / this->omega_max *
                                         (this->K1 * sigma_i +
                                          this->K3 * sigma_i * sigma_i * sigma_i),
                                     2.0) +
                            1.0);
            omega_ast_p[i] = -value * sigma_p[i];
        }
    }

    eigenVector3d2CArray(omega_ast, outMsg.omega_BastR_B);
    eigenVector3d2CArray(omega_ast_p, outMsg.omegap_BastR_B);
    return outMsg;
}

double MrpSteeringAlgorithm::getK1() const { return this->K1; }
double MrpSteeringAlgorithm::getK3() const { return this->K3; }
double MrpSteeringAlgorithm::getOmegaMax() const { return this->omega_max; }
bool MrpSteeringAlgorithm::getIgnoreOuterLoopFeedforward() const { return this->ignoreOuterLoopFeedforward; }
void MrpSteeringAlgorithm::setK1(double K1) { this->K1 = K1; }
void MrpSteeringAlgorithm::setK3(double K3) { this->K3 = K3; }
void MrpSteeringAlgorithm::setOmegaMax(double omegaMax) { this->omega_max = omegaMax; }
void MrpSteeringAlgorithm::setIgnoreOuterLoopFeedforward(bool ignore) { this->ignoreOuterLoopFeedforward = ignore; }
