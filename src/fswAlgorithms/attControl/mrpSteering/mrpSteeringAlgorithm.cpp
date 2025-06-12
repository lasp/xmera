#include "fswAlgorithms/attControl/mrpSteering/mrpSteeringAlgorithm.h"

#include "architecture/utilities/avsEigenSupport.h"
#include "architecture/utilities/rigidBodyKinematics.hpp"
#include <cassert>
#include <cmath>

void MrpSteeringAlgorithm::reset(uint64_t /*callTime*/) {}

RateCmdMsgPayload MrpSteeringAlgorithm::update(uint64_t /*callTime*/, AttGuidMsgPayload &guidCmd) {
    Eigen::Vector3d sigma_BR = cArray2EigenVector3d(guidCmd.sigma_BR);

    Eigen::Vector3d omega_ast = Eigen::Vector3d::Zero();
    Eigen::Vector3d omega_ast_p = Eigen::Vector3d::Zero();

    for (int i = 0; i < 3; ++i) {
        double sigma_i = sigma_BR(i);
        double value = std::atan((M_PI_2 / this->omega_max) *
                                 (this->K1 * sigma_i + this->K3 * sigma_i * sigma_i * sigma_i)) /
                       M_PI_2 * this->omega_max;
        omega_ast(i) = -value;
    }

    if (!this->ignoreOuterLoopFeedforward) {
        Eigen::Matrix3d B = bmatMrp(sigma_BR);
        Eigen::Vector3d sigma_p = 0.25 * B * omega_ast;

        for (int i = 0; i < 3; ++i) {
            double sigma_i = sigma_BR(i);
            double denom = std::pow((M_PI_2 / this->omega_max) *
                                        (this->K1 * sigma_i + this->K3 * sigma_i * sigma_i * sigma_i),
                                    2.0) +
                            1.0;
            double value = (3.0 * this->K3 * sigma_i * sigma_i + this->K1) / denom;
            omega_ast_p(i) = -value * sigma_p(i);
        }
    }

    RateCmdMsgPayload out{};
    eigenVector3d2CArray(omega_ast, out.omega_BastR_B);
    eigenVector3d2CArray(omega_ast_p, out.omegap_BastR_B);

    return out;
}

void MrpSteeringAlgorithm::setK1(double k1) { this->K1 = k1; }
double MrpSteeringAlgorithm::getK1() const { return this->K1; }
void MrpSteeringAlgorithm::setK3(double k3) { this->K3 = k3; }
double MrpSteeringAlgorithm::getK3() const { return this->K3; }
void MrpSteeringAlgorithm::setOmegaMax(double omegaMax) { this->omega_max = omegaMax; }
double MrpSteeringAlgorithm::getOmegaMax() const { return this->omega_max; }
void MrpSteeringAlgorithm::setIgnoreOuterLoopFeedforward(bool flag) { this->ignoreOuterLoopFeedforward = flag; }
bool MrpSteeringAlgorithm::getIgnoreOuterLoopFeedforward() const { return this->ignoreOuterLoopFeedforward; }


