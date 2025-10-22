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

#include "celestialTwoBodyPointAlgorithm.h"
#include <architecture/utilities/eigenSupport.h>
#include <architecture/utilities/rigidBodyKinematics.hpp>
#include <architecture/utilities/safeMath.h>

#include <math.h>

void CelestialTwoBodyPointAlgorithm::reset(bool secCelBodyIsLinked) { this->secCelBodyIsLinked = secCelBodyIsLinked; }

/*! This method takes the spacecraft and points a specified axis at a named
 celestial body specified in the configuration data.  It generates the
 commanded attitude and assumes that the control errors are computed
 downstream.
 @return AttRefMsgPayload
 */
AttRefMsgPayload CelestialTwoBodyPointAlgorithm::update(EphemerisMsgPayload& celBodyIn,
                                                        EphemerisMsgPayload& secCelBodyIn,
                                                        NavTransMsgPayload& transNavIn) {
    double platAngDiff{}; /* Angle between r_P1 and r_P2 */

    Eigen::Vector3d R_P1B_N =
        Eigen::Map<const Eigen::Vector3d>(celBodyIn.r_BdyZero_N) - Eigen::Map<const Eigen::Vector3d>(transNavIn.r_BN_N);
    Eigen::Vector3d v_P1B_N =
        Eigen::Map<const Eigen::Vector3d>(celBodyIn.v_BdyZero_N) - Eigen::Map<const Eigen::Vector3d>(transNavIn.v_BN_N);

    Eigen::Vector3d R_P2B_N{};
    Eigen::Vector3d v_P2B_N{};

    if (this->secCelBodyIsLinked) {
        R_P2B_N = Eigen::Map<const Eigen::Vector3d>(secCelBodyIn.r_BdyZero_N) -
                  Eigen::Map<const Eigen::Vector3d>(transNavIn.r_BN_N);
        v_P2B_N = Eigen::Map<const Eigen::Vector3d>(secCelBodyIn.v_BdyZero_N) -
                  Eigen::Map<const Eigen::Vector3d>(transNavIn.v_BN_N);
        double dotProduct = R_P2B_N.normalized().dot(R_P1B_N.normalized());
        platAngDiff = safeAcos(dotProduct);
    }

    /*! - Cross the P1 states to get R_P2, v_p2 and a_P2 */
    if (!this->secCelBodyIsLinked || fabs(platAngDiff) < this->singularityThresh ||
        fabs(platAngDiff) > M_PI - this->singularityThresh) {
        R_P2B_N = R_P1B_N.cross(v_P1B_N);
        v_P2B_N = Eigen::Vector3d::Zero();
    }

    AttRefMsgPayload attRefOut{};

    /* - Initial computations: R_n, v_n, a_n */
    Eigen::Vector3d R_N = R_P1B_N.cross(R_P2B_N);
    Eigen::Vector3d v_N = v_P1B_N.cross(R_P2B_N) + R_P1B_N.cross(v_P2B_N);
    Eigen::Vector3d a_N = 2 * v_P1B_N.cross(v_P2B_N);

    /* - Reference Frame computation */
    Eigen::Vector3d r1_N_hat = R_P1B_N.normalized();
    Eigen::Vector3d r3_N_hat = R_N.normalized();
    Eigen::Vector3d r2_N_hat = (r3_N_hat.cross(r1_N_hat)).normalized();
    Eigen::Matrix3d dcm_RN{};
    dcm_RN.row(0) = r1_N_hat;
    dcm_RN.row(1) = r2_N_hat;
    dcm_RN.row(2) = r3_N_hat;

    /* - MRP computation */
    Eigen::Vector3d sigma_RN = dcmToMrp(dcm_RN);
    eigenVectorToCArray(sigma_RN, attRefOut.sigma_RN);

    /* - Reference base-vectors first time-derivative */
    Eigen::Vector3d dr1_N_hat =
        (Eigen::Matrix3d::Identity() - r1_N_hat * r1_N_hat.transpose()) * v_P1B_N / R_P1B_N.norm();
    Eigen::Vector3d dr3_N_hat = (Eigen::Matrix3d::Identity() - r3_N_hat * r3_N_hat.transpose()) * v_N / R_N.norm();
    Eigen::Vector3d dr2_N_hat = dr3_N_hat.cross(r1_N_hat) + r3_N_hat.cross(dr1_N_hat);

    /* - Angular velocity computation */
    Eigen::Vector3d omega_RN_R{};
    omega_RN_R[0] = r3_N_hat.dot(dr2_N_hat);
    omega_RN_R[1] = r1_N_hat.dot(dr3_N_hat);
    omega_RN_R[2] = r2_N_hat.dot(dr1_N_hat);
    Eigen::Vector3d omega_RN_N = dcm_RN.transpose() * omega_RN_R;
    eigenVectorToCArray(omega_RN_N, attRefOut.omega_RN_N);

    /* - Reference base-vectors second time-derivative */
    Eigen::Vector3d ddr1_N_hat =
        -(2 * dr1_N_hat * r1_N_hat.transpose() + r1_N_hat * dr1_N_hat.transpose()) * v_P1B_N / R_P1B_N.norm();
    Eigen::Vector3d ddr3_N_hat = ((Eigen::Matrix3d::Identity() - r3_N_hat * r3_N_hat.transpose()) * a_N -
                                  (2 * dr3_N_hat * r3_N_hat.transpose() + r3_N_hat * dr3_N_hat.transpose()) * v_N) /
                                 R_N.norm();
    Eigen::Vector3d ddr2_N_hat =
        ddr3_N_hat.cross(r1_N_hat) + r3_N_hat.cross(ddr1_N_hat) + 2 * dr3_N_hat.cross(dr1_N_hat);

    /* - Angular acceleration computation */
    Eigen::Vector3d domega_RN_R{};
    domega_RN_R[0] = dr3_N_hat.dot(dr2_N_hat) + r3_N_hat.dot(ddr2_N_hat) - omega_RN_R.dot(dr1_N_hat);
    domega_RN_R[1] = dr1_N_hat.dot(dr3_N_hat) + r1_N_hat.dot(ddr3_N_hat) - omega_RN_R.dot(dr2_N_hat);
    domega_RN_R[2] = dr2_N_hat.dot(dr1_N_hat) + r2_N_hat.dot(ddr1_N_hat) - omega_RN_R.dot(dr3_N_hat);
    Eigen::Vector3d domega_RN_N = dcm_RN.transpose() * domega_RN_R;
    eigenVectorToCArray(domega_RN_N, attRefOut.domega_RN_N);

    return attRefOut;
}

/**
 * @brief Set the singularity threshold
 * @param thresh singularity threshold
 */
void CelestialTwoBodyPointAlgorithm::setSingularityThresh(double thresh) {
    if (thresh < 0.0) {
        throw std::invalid_argument("Singularity threshold must not be negative");
    }
    this->singularityThresh = thresh;
}

/**
 * @brief Get the singularity threshold
 * @return double singularity threshold
 */
double CelestialTwoBodyPointAlgorithm::getSingularityThresh() const { return this->singularityThresh; }
