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

#include "sunlineEphemAlgorithm.h"
NavAttMsgPayload SunlineEphemAlgorithm::updateState(uint64_t callTime,
                                                    const EphemerisMsgPayload &sunPos,
                                                    const NavTransMsgPayload &scPos,
                                                    const NavAttMsgPayload &scAtt) {
    assert(!sunPos.isLinked());
    assert(!scPos.isLinked());
    assert(!scAtt.isLinked());
    // Get sun position
    const Eigen::Vector3d rSun(sunPos.r_BdyZero_N[0], sunPos.r_BdyZero_N[1], sunPos.r_BdyZero_N[2]);
    const Eigen::Vector3d rSc(scPos.r_BN_N[0], scPos.r_BN_N[1], scPos.r_BN_N[2]);
    // Difference in inertial frame
    const Eigen::Vector3d r_SB_N = rSun - rSc;
    Eigen::Vector3d r_SB_N_hat = Eigen::Vector3d::Zero();
    if (r_SB_N.norm() > std::numeric_limits<double>::epsilon()) {
        r_SB_N_hat = r_SB_N;
        r_SB_N_hat.normalize();  // in-place unit-length
    }
    // Build DCM from spacecraft attitude
    const Eigen::Vector3d sigma_BN(scAtt.sigma_BN[0], scAtt.sigma_BN[1], scAtt.sigma_BN[2]);
    const Eigen::Matrix3d dcm_BN = mrpToDcm(sigma_BN);

    // Rotate into body frame
    Eigen::Vector3d r_SB_B_hat = dcm_BN * r_SB_N_hat;

    // Ensure unit length (or zero)
    if (r_SB_B_hat.norm() > std::numeric_limits<double>::epsilon()) {
        r_SB_B_hat.normalize();  // in-place unit-length
    } else {
        r_SB_B_hat.setZero();  // explicit zero
    }

    auto outputSunline = NavAttMsgPayload{}; /* [-] Output sunline estimate data */
    for (int i = 0; i < 3; i++) {
        outputSunline.vehSunPntBdy[i] = r_SB_B_hat[i];
    }
    return outputSunline;
}
