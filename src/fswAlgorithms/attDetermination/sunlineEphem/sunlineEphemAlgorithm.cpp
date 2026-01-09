#include "sunlineEphemAlgorithm.h"
NavAttMsgPayload SunlineEphemAlgorithm::updateState(const EphemerisMsgPayload& sunPos,
                                                    const NavTransMsgPayload& scPos,
                                                    const NavAttMsgPayload& scAtt) const {
    // Get sun position
    const Eigen::Vector3d r_SN_N(sunPos.r_BdyZero_N[0], sunPos.r_BdyZero_N[1], sunPos.r_BdyZero_N[2]);
    const Eigen::Vector3d r_BN_N(scPos.r_BN_N[0], scPos.r_BN_N[1], scPos.r_BN_N[2]);
    // Difference in inertial frame
    const Eigen::Vector3d r_SB_N = r_SN_N - r_BN_N;
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
