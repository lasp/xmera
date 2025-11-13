#ifndef RIGIDBODYKINEMATICS_HPP
#define RIGIDBODYKINEMATICS_HPP

#include <Eigen/Dense>

#include <limits>

template <typename ScalarT>
struct Types {
    static constexpr ScalarT eps = std::numeric_limits<ScalarT>::epsilon();
};

/**
 *  Switch from the current mrp to its shadow set
 * @param mrp
 * @return Eigen::Vector3d
 */
template <typename ScalarT>
Eigen::Vector3<ScalarT> mrpShadow(const Eigen::Vector3<ScalarT>& mrp) {
    return -mrp / mrp.squaredNorm();
}

/**
 * Check if mrp norm is larger than s. If so, map mrp to its shadow set.
 * @param mrp
 * @param s
 * @return Eigen::Vector3d
 */
template <typename ScalarT>
Eigen::Vector3<ScalarT> mrpSwitch(const Eigen::Vector3<ScalarT>& mrp, const ScalarT s) {
    if (mrp.squaredNorm() > s * s) {
        return mrpShadow(mrp);
    }
    return mrp;
}

/**
 * Return the matrix which relates the derivative of Euler parameter vector to the body angular
 * velocity vector omega = 2 [B(Q)]^(-1) dQ/dt
 * @param ep const Eigen::Vector4
 * @return Eigen::Matrix<double, 3, 4>
 */
template <typename ScalarT>
Eigen::Matrix<ScalarT, 3, 4> binvEp(const Eigen::Vector4<ScalarT>& ep) {
    Eigen::Matrix<ScalarT, 3, 4> Binv;

    Binv << -ep(1), ep(0), ep(3), -ep(2), -ep(2), -ep(3), ep(0), ep(1), -ep(3), ep(2), -ep(1), ep(0);

    return Binv;
}

/**
 * Return the matrix which relates the derivative of mrp vector Q to the body angular velocity vector
 * omega = 4 [B(Q)]^(-1) dQ/dt
 * @param mrp Eigen::Vector3d
 * @return Eigen::Matrix3d
 */
template <typename ScalarT>
Eigen::Matrix3<ScalarT> binvMrp(const Eigen::Vector3<ScalarT>& mrp) {
    Eigen::Matrix3<ScalarT> Binv;

    ScalarT dotProd = mrp.dot(mrp);
    ScalarT denom = (1 + dotProd) * (1 + dotProd);

    Binv(0, 0) = 1 - dotProd + 2 * mrp(0) * mrp(0);
    Binv(0, 1) = 2 * (mrp(0) * mrp(1) + mrp(2));
    Binv(0, 2) = 2 * (mrp(0) * mrp(2) - mrp(1));
    Binv(1, 0) = 2 * (mrp(1) * mrp(0) - mrp(2));
    Binv(1, 1) = 1 - dotProd + 2 * mrp(1) * mrp(1);
    Binv(1, 2) = 2 * (mrp(1) * mrp(2) + mrp(0));
    Binv(2, 0) = 2 * (mrp(2) * mrp(0) + mrp(1));
    Binv(2, 1) = 2 * (mrp(2) * mrp(1) - mrp(0));
    Binv(2, 2) = 1 - dotProd + 2 * mrp(2) * mrp(2);

    return Binv / denom;
}

/**
 * Return the matrix which relates the derivative of principal rotation vector to the body angular velocity vector
 * omega = [B(Q)]^(-1) dQ/dt
 * @param prv Eigen::Vector3
 * @return Eigen::Matrix3d
 */
template <typename ScalarT>
Eigen::Matrix3<ScalarT> binvPrv(const Eigen::Vector3<ScalarT>& prv) {
    ScalarT norm = prv.norm();
    ScalarT norm2 = norm * norm;
    ScalarT norm3 = norm2 * norm;

    ScalarT c1 = (1.0 - std::cos(norm)) / norm2;
    ScalarT c2 = (norm - std::sin(norm)) / norm3;

    Eigen::Matrix3<ScalarT> Binv;
    Binv(0, 0) = 1.0 - c2 * (prv(1) * prv(1) + prv(2) * prv(2));
    Binv(0, 1) = c1 * prv(2) + c2 * prv(0) * prv(1);
    Binv(0, 2) = -c1 * prv(1) + c2 * prv(0) * prv(2);

    Binv(1, 0) = -c1 * prv(2) + c2 * prv(0) * prv(1);
    Binv(1, 1) = 1.0 - c2 * (prv(0) * prv(0) + prv(2) * prv(2));
    Binv(1, 2) = c1 * prv(0) + c2 * prv(1) * prv(2);

    Binv(2, 0) = c1 * prv(1) + c2 * prv(2) * prv(0);
    Binv(2, 1) = -c1 * prv(0) + c2 * prv(2) * prv(1);
    Binv(2, 2) = 1.0 - c2 * (prv(0) * prv(0) + prv(1) * prv(1));

    return Binv;
}

/**
 * Return the matrix which relates the derivative of a 321 Euler angle vector to the body angular velocity vector.
 * omega = [B(euler321)]^(-1) dQ/dt
 * @param euler321 Eigen::Vector3d
 * @return Eigen::Matrix3d
 */
template <typename ScalarT>
Eigen::Matrix3<ScalarT> binvEulerAngles321(const Eigen::Vector3<ScalarT>& euler321) {
    ScalarT sin2 = std::sin(euler321(1));
    ScalarT cos2 = std::cos(euler321(1));
    ScalarT sin3 = std::sin(euler321(2));
    ScalarT cos3 = std::cos(euler321(2));

    Eigen::Matrix3<ScalarT> Binv;
    Binv << -sin2, ScalarT(0), ScalarT(1), cos2 * sin3, cos3, ScalarT(0), cos2 * cos3, -sin3, ScalarT(0);

    return Binv;
}

/**
 * Return the matrix which relates the body angular velocity vector to the derivative of Euler parameter vector
 * dQ/dt = 1/2 [B(Q)] omega
 * @param ep Eigen::Vector4d
 * @return Eigen::Matrix<double, 4, 3>
 */
template <typename ScalarT>
Eigen::Matrix<ScalarT, 4, 3> bmatEp(const Eigen::Vector4<ScalarT>& ep) {
    Eigen::Matrix<ScalarT, 4, 3> B;

    B(0, 0) = -ep(1);
    B(0, 1) = -ep(2);
    B(0, 2) = -ep(3);
    B(1, 0) = ep(0);
    B(1, 1) = -ep(3);
    B(1, 2) = ep(2);
    B(2, 0) = ep(3);
    B(2, 1) = ep(0);
    B(2, 2) = -ep(1);
    B(3, 0) = -ep(2);
    B(3, 1) = ep(1);
    B(3, 2) = ep(0);

    return B;
}

/**
 * Return the matrix which relates the body angular velocity vector w to the derivative of mrp vector
 * dQ/dt = 1/4 [B(Q)] omega
 * @param mrp
 * @return Eigen::Matrix3d
 */
template <typename ScalarT>
Eigen::Matrix3<ScalarT> bmatMrp(const Eigen::Vector3<ScalarT>& mrp) {
    Eigen::Matrix3<ScalarT> B;

    B(0, 0) = 1 - mrp.dot(mrp) + 2 * mrp(0) * mrp(0);
    B(0, 1) = 2 * (mrp(0) * mrp(1) - mrp(2));
    B(0, 2) = 2 * (mrp(0) * mrp(2) + mrp(1));
    B(1, 0) = 2 * (mrp(1) * mrp(0) + mrp(2));
    B(1, 1) = 1 - mrp.dot(mrp) + 2 * mrp(1) * mrp(1);
    B(1, 2) = 2 * (mrp(1) * mrp(2) - mrp(0));
    B(2, 0) = 2 * (mrp(2) * mrp(0) - mrp(1));
    B(2, 1) = 2 * (mrp(2) * mrp(1) + mrp(0));
    B(2, 2) = 1 - mrp.dot(mrp) + 2 * mrp(2) * mrp(2);

    return B;
}

/**
 * Return the matrix derivative of the bmatMrp matrix, and it is used to relate the  body angular acceleration
 * vector dw to the second order derivative of the mrp vector
 * (d^2Q)/(dt^2) = 1/4 ( [B(Q)] dw + [Bdot(Q,dQ)] w )
 * @param mrp
 * @param dmrp
 * @return Eigen::Matrix3d
 */
template <typename ScalarT>
Eigen::Matrix3<ScalarT> bmatDotMrp(const Eigen::Vector3<ScalarT>& mrp, const Eigen::Vector3<ScalarT>& dmrp) {
    Eigen::Matrix3<ScalarT> B;

    B(0, 0) = -2 * mrp.dot(dmrp) + 4 * (mrp(0) * dmrp(0));
    B(0, 1) = 2 * (-dmrp(2) + mrp(0) * dmrp(1) + dmrp(0) * mrp(1));
    B(0, 2) = 2 * (dmrp(1) + mrp(0) * dmrp(2) + dmrp(0) * mrp(2));
    B(1, 0) = 2 * (dmrp(2) + mrp(0) * dmrp(1) + dmrp(0) * mrp(1));
    B(1, 1) = -2 * mrp.dot(dmrp) + 4 * (mrp(1) * dmrp(1));
    B(1, 2) = 2 * (-dmrp(0) + mrp(1) * dmrp(2) + dmrp(1) * mrp(2));
    B(2, 0) = 2 * (-dmrp(1) + mrp(0) * dmrp(2) + dmrp(0) * mrp(2));
    B(2, 1) = 2 * (dmrp(0) + mrp(1) * dmrp(2) + dmrp(1) * mrp(2));
    B(2, 2) = -2 * mrp.dot(dmrp) + 4 * (mrp(2) * dmrp(2));

    return B;
}

/**
 * Return the matrix which relates the  body angular velocity vector to the derivative of principal
 * rotation vector dQ/dt = [B(Q)] omega
 * @param prv
 * @return Eigen::Matrix3d
 */
template <typename ScalarT>
Eigen::Matrix3<ScalarT> bmatPrv(const Eigen::Vector3<ScalarT>& prv) {
    ScalarT normPrv = prv.norm();
    ScalarT c = 1.0 / (normPrv * normPrv) * (1.0 - normPrv / 2.0 / std::tan(normPrv / 2.0));

    Eigen::Matrix3<ScalarT> B;
    B(0, 0) = 1.0 - c * (prv(1) * prv(1) + prv(2) * prv(2));
    B(0, 1) = -prv(2) / 2.0 + c * (prv(0) * prv(1));
    B(0, 2) = prv(1) / 2.0 + c * (prv(0) * prv(2));
    B(1, 0) = prv(2) / 2.0 + c * (prv(0) * prv(1));
    B(1, 1) = 1.0 - c * (prv(0) * prv(0) + prv(2) * prv(2));
    B(1, 2) = -prv(0) / 2.0 + c * (prv(1) * prv(2));
    B(2, 0) = -prv(1) / 2.0 + c * (prv(0) * prv(2));
    B(2, 1) = prv(0) / 2.0 + c * (prv(1) * prv(2));
    B(2, 2) = 1.0 - c * (prv(0) * prv(0) + prv(1) * prv(1));

    return B;
}

/**
 * Return the matrix which relates the body angular velocity vector to the derivative of 321 Euler angle vector.
 * dQ/dt = [B(Q)] omega
 * @param euler321
 * @return Eigen::Matrix3d
 */
template <typename ScalarT>
Eigen::Matrix3<ScalarT> bmatEulerAngles321(const Eigen::Vector3<ScalarT>& euler321) {
    ScalarT sin2 = std::sin(euler321(1));
    ScalarT cos2 = std::cos(euler321(1));
    ScalarT sin3 = std::sin(euler321(2));
    ScalarT cos3 = std::cos(euler321(2));

    Eigen::Matrix3<ScalarT> B;
    B << 0.0, sin3, cos3, 0.0, cos2 * cos3, -cos2 * sin3, cos2, sin2 * sin3, sin2 * cos3;

    return B / cos2;
}

/**
 * Translate the direction cosine matrix into the corresponding Euler parameter vector,
 * where the first component of is the non-dimensional Euler parameter >= 0. Transformation is done
 * using the Stanley method.
 * @param dcm
 * @return Eigen::Vector4d
 */
template <typename ScalarT>
Eigen::Vector4<ScalarT> dcmToEp(const Eigen::Matrix3<ScalarT>& dcm) {
    Eigen::Vector4<ScalarT> ep;

    ep(0) = (1.0 + dcm.trace()) / 4.0;
    ep(1) = (1.0 + 2.0 * dcm(0, 0) - dcm.trace()) / 4.0;
    ep(2) = (1.0 + 2.0 * dcm(1, 1) - dcm.trace()) / 4.0;
    ep(3) = (1.0 + 2.0 * dcm(2, 2) - dcm.trace()) / 4.0;

    if (ScalarT maxVal = ep.maxCoeff(); maxVal == ep(0)) {
        ep(0) = std::sqrt(ep(0));
        ep(1) = (dcm(1, 2) - dcm(2, 1)) / (4.0 * ep(0));
        ep(2) = (dcm(2, 0) - dcm(0, 2)) / (4.0 * ep(0));
        ep(3) = (dcm(0, 1) - dcm(1, 0)) / (4.0 * ep(0));
    } else if (maxVal == ep(1)) {
        ep(1) = std::sqrt(ep(1));
        ep(0) = (dcm(1, 2) - dcm(2, 1)) / (4.0 * ep(1));
        if (ep(0) < 0.0) {
            ep(1) = -ep(1);
            ep(0) = -ep(0);
        }
        ep(2) = (dcm(0, 1) + dcm(1, 0)) / (4.0 * ep(1));
        ep(3) = (dcm(2, 0) + dcm(0, 2)) / (4.0 * ep(1));
    } else if (maxVal == ep(2)) {
        ep(2) = std::sqrt(ep(2));
        ep(0) = (dcm(2, 0) - dcm(0, 2)) / (4.0 * ep(2));
        if (ep(0) < 0.0) {
            ep(2) = -ep(2);
            ep(0) = -ep(0);
        }
        ep(1) = (dcm(0, 1) + dcm(1, 0)) / (4.0 * ep(2));
        ep(3) = (dcm(1, 2) + dcm(2, 1)) / (4.0 * ep(2));
    } else if (maxVal == ep(3)) {
        ep(3) = std::sqrt(ep(3));
        ep(0) = (dcm(0, 1) - dcm(1, 0)) / (4.0 * ep(3));
        if (ep(0) < 0.0) {
            ep(3) = -ep(3);
            ep(0) = -ep(0);
        }
        ep(1) = (dcm(2, 0) + dcm(0, 2)) / (4.0 * ep(3));
        ep(2) = (dcm(1, 2) + dcm(2, 1)) / (4.0 * ep(3));
    }

    return ep;
}

/**
 * Translate a direction cosine matrix into the corresponding mrp vector where the mrp vector is chosen such
 * that its norm is less than 1
 * @param dcm
 * @return Eigen::Vector3d
 */
template <typename ScalarT>
Eigen::Vector3<ScalarT> dcmToMrp(const Eigen::Matrix<ScalarT, 3, 3>& dcm) {
    Eigen::Vector4<ScalarT> ep = dcmToEp(dcm);
    return ep.template tail<3>() / (1.0 + ep(0));
}

/**
 * Translates a Euler parameter vector into a principal rotation vector.
 * @param ep
 * @return Eigen::Vector3d
 */
template <typename ScalarT>
Eigen::Vector3<ScalarT> epToPrv(const Eigen::Vector4<ScalarT>& ep, ScalarT localEps = ScalarT(1e-12)) {
    Eigen::Vector3<ScalarT> prv;
    ScalarT angle = std::acos(ep(0));
    ScalarT sin_angle = std::sin(angle);
    if (std::abs(sin_angle) < localEps) {
        return prv.setZero();
    }
    prv = ep.template tail<3>() / sin_angle * 2.0 * angle;
    return prv;
}

/**
 * Translate a direction cosine matrix into the corresponding principal rotation vector,
 * where the first component is the principal rotation angle 0<= phi <= Pi
 * @param dcm
 * @return Eigen::Vector3d
 */
template <typename ScalarT>
Eigen::Vector3<ScalarT> dcmToPrv(const Eigen::Matrix3<ScalarT>& dcm) {
    return epToPrv(dcmToEp(dcm));
}

/**
 * Translate a direction cosine matrix into the corresponding 321 Euler angle set.
 * @param dcm
 * @return Eigen::Vector3d
 */
template <typename ScalarT>
Eigen::Vector3<ScalarT> dcmToEulerAngles321(const Eigen::Matrix3<ScalarT>& dcm) {
    Eigen::Vector3<ScalarT> euler321;

    euler321[0] = std::atan2(dcm(0, 1), dcm(0, 0));
    euler321[1] = std::asin(-dcm(0, 2));
    euler321[2] = std::atan2(dcm(1, 2), dcm(2, 2));

    return euler321;
}

/**
 * Return the Euler parameter derivative for a given Euler parameter vector and body
 * angular velocity vector omega, dQ/dt = 1/2 [B(Q)] omega
 * @param ep
 * @param omega
 * @return Eigen::Vector4d
 */
template <typename ScalarT>
Eigen::Vector4<ScalarT> dep(const Eigen::Vector4<ScalarT>& ep, const Eigen::Vector3<ScalarT>& omega) {
    Eigen::Matrix<ScalarT, 4, 3> B = bmatEp(ep);
    Eigen::Vector4<ScalarT> dep = B * omega;
    return dep / 2.0;
}

/**
 * Return the mrp derivative for a given mrp vector and body angular velocity vector.
 * dQ/dt = 1/4 [B(Q)] omega
 * @param mrp
 * @param omega
 * @return Eigen::Vector3d
 */
template <typename ScalarT>
Eigen::Vector3<ScalarT> dmrp(const Eigen::Vector3<ScalarT>& mrp, const Eigen::Vector3<ScalarT>& omega) {
    Eigen::Matrix3<ScalarT> B = bmatMrp(mrp);
    return B * omega / 4.0;
}

/**
 * Return the angular rate for a given mrp vector and mrp derivative.
 * omega = 4 [B(Q)]^(-1) dQ/dt
 * @param mrp
 * @param dmrp
 * @return Eigen::Vector3d
 */
template <typename ScalarT>
Eigen::Vector3<ScalarT> dmrpToOmega(const Eigen::Vector3<ScalarT>& mrp, const Eigen::Vector3<ScalarT>& dmrp) {
    Eigen::Matrix3<ScalarT> Binv = binvMrp(mrp);
    return 4.0 * Binv * dmrp;
}

/**
 * Return the second order mrp derivative for a given mrp vector, first mrp derivative, body angular
 * velocity vector and body angular acceleration vector.
 * (d^2Q)/(dt^2) = 1/4 ( [B(Q)] dw + [Bdot(Q,dQ)] w )
 * @param mrp
 * @param dmrp
 * @param omega
 * @param domega
 * @return Eigen::Vector3d
 */
template <typename ScalarT>
Eigen::Vector3<ScalarT> ddmrp(const Eigen::Vector3<ScalarT>& mrp,
                              const Eigen::Vector3<ScalarT>& dmrp,
                              const Eigen::Vector3<ScalarT>& omega,
                              const Eigen::Vector3<ScalarT>& domega) {
    Eigen::Matrix3<ScalarT> B = bmatMrp(mrp);
    Eigen::Matrix3<ScalarT> Bdot = bmatDotMrp(mrp, dmrp);

    return (B * domega + Bdot * omega) / 4.0;
}

/**
 * Return the angular rate for a given mrp vector and mrp derivative.
 * domega/dt = 4 [B(Q)]^(-1) ( ddQ - [Bdot(Q,dQ)] [B(Q)]^(-1) dQ )
 * @param mrp
 * @param dmrp
 * @param ddmrp
 * @return Eigen::Vector3d
 */
template <typename ScalarT>
Eigen::Vector3<ScalarT> ddmrpTodOmega(const Eigen::Vector3<ScalarT>& mrp,
                                      const Eigen::Vector3<ScalarT>& dmrp,
                                      const Eigen::Vector3<ScalarT>& ddmrp) {
    Eigen::Matrix3<ScalarT> Binv = binvMrp(mrp);
    Eigen::Matrix3<ScalarT> Bdot = bmatDotMrp(mrp, dmrp);
    Eigen::Vector3<ScalarT> diff = ddmrp - Bdot * Binv * dmrp;
    return 4.0 * Binv * diff;
}

/**Return the PRV derivative for a given PRV vector and body angular velocity vector.
 * dQ/dt =  [B(Q)] omega
 * @param prv
 * @param omega
 * @return Eigen::Vector3d
 */
template <typename ScalarT>
Eigen::Vector3<ScalarT> dprv(const Eigen::Vector3<ScalarT>& prv, const Eigen::Vector3<ScalarT>& omega) {
    return bmatPrv(prv) * omega;
}

/**
 * Return the 321 Euler angle derivative vector for a given 321 Euler angle vector and body
 * angular velocity vector.
 * dQ/dt =  [B(Q)] omega
 * @param euler321
 * @param omega
 * @return Eigen::Vector3d
 */
template <typename ScalarT>
Eigen::Vector3<ScalarT> deuler321(const Eigen::Vector3<ScalarT>& euler321, const Eigen::Vector3<ScalarT>& omega) {
    return bmatEulerAngles321(euler321) * omega;
}

/**
 * Return the direction cosine matrix in terms of the Euler parameter vector. The first element is the
 * non-dimensional Euler parameter, while the remain three elements form the Euler parameter vector.
 * @param ep
 * @return Eigen::Matrix3d
 */
template <typename ScalarT>
Eigen::Matrix3<ScalarT> epToDcm(const Eigen::Vector4<ScalarT>& ep) {
    Eigen::Matrix3<ScalarT> dcm;

    dcm(0, 0) = ep(0) * ep(0) + ep(1) * ep(1) - ep(2) * ep(2) - ep(3) * ep(3);
    dcm(0, 1) = 2.0 * (ep(1) * ep(2) + ep(0) * ep(3));
    dcm(0, 2) = 2.0 * (ep(1) * ep(3) - ep(0) * ep(2));
    dcm(1, 0) = 2.0 * (ep(1) * ep(2) - ep(0) * ep(3));
    dcm(1, 1) = ep(0) * ep(0) - ep(1) * ep(1) + ep(2) * ep(2) - ep(3) * ep(3);
    dcm(1, 2) = 2.0 * (ep(2) * ep(3) + ep(0) * ep(1));
    dcm(2, 0) = 2.0 * (ep(1) * ep(3) + ep(0) * ep(2));
    dcm(2, 1) = 2.0 * (ep(2) * ep(3) - ep(0) * ep(1));
    dcm(2, 2) = ep(0) * ep(0) - ep(1) * ep(1) - ep(2) * ep(2) + ep(3) * ep(3);

    return dcm;
}

/**
 * Translate a Euler parameter vector into an mrp vector.
 * @param ep
 * @return Eigen::Vector3d
 */
template <typename ScalarT>
Eigen::Vector3<ScalarT> epToMrp(const Eigen::Vector4<ScalarT>& ep) {
    if (ep(0) >= 0.0) {
        return ep.template tail<3>() / (1.0 + ep(0));
    } else {
        return -ep.template tail<3>() / (1.0 - ep(0));
    }
}

/**
 * Translate a Euler parameter vector into the corresponding 321 Euler angle set.
 * @param ep
 * @return Eigen::Vector3d
 */
template <typename ScalarT>
Eigen::Vector3<ScalarT> epToEulerAngles321(const Eigen::Vector4<ScalarT>& ep) {
    Eigen::Vector3<ScalarT> euler321;

    euler321(0) = std::atan2(2.0 * (ep(1) * ep(2) + ep(0) * ep(3)),
                             ep(0) * ep(0) + ep(1) * ep(1) - ep(2) * ep(2) - ep(3) * ep(3));
    euler321(1) = std::asin(-2.0 * (ep(1) * ep(3) - ep(0) * ep(2)));
    euler321(2) = std::atan2(2.0 * (ep(2) * ep(3) + ep(0) * ep(1)),
                             ep(0) * ep(0) - ep(1) * ep(1) - ep(2) * ep(2) + ep(3) * ep(3));

    return euler321;
}

/**
 * Return the the cross product matrix
 * @param vector
 * @return Eigen::Matrix3d
 */
template <typename ScalarT>
Eigen::Matrix3<ScalarT> tildeMatrix(const Eigen::Vector3<ScalarT>& vector) {
    Eigen::Matrix3<ScalarT> tilde;
    tilde << ScalarT(0), -vector(2), vector(1), vector(2), ScalarT(0), -vector(0), -vector(1), vector(0), ScalarT(0);
    return tilde;
}

/**
 * Return the direction cosine matrix in terms of an mrp vector.
 * @param mrp
 * @return Eigen::Matrix3d
 */
template <typename ScalarT>
Eigen::Matrix3<ScalarT> mrpToDcm(const Eigen::Vector3<ScalarT>& mrp) {
    Eigen::Matrix3<ScalarT> dcm;

    Eigen::Matrix3<ScalarT> t = tildeMatrix(mrp);
    ScalarT denom = (1.0 + mrp.dot(mrp));
    dcm = (8.0 * t * t - 4.0 * (1.0 - mrp.dot(mrp)) * t) / (denom * denom);

    return dcm + Eigen::Matrix3<ScalarT>::Identity();
}

/**
 * Translate the mrp vector into the Euler parameter vector.
 * @param mrp
 * @return Eigen::Vector4d
 */
template <typename ScalarT>
Eigen::Vector4<ScalarT> mrpToEp(const Eigen::Vector3<ScalarT>& mrp) {
    Eigen::Vector4<ScalarT> ep;
    ScalarT dotVal = mrp.dot(mrp);

    ep(0) = 1.0 - dotVal;
    ep.template tail<3>() = 2.0 * mrp;

    return ep / (1.0 + dotVal);
}

/**
 * Translate a mrp vector into a principal rotation vector
 * @param mrp
 * @return Eigen::Vector3d
 */
template <typename ScalarT>
Eigen::Vector3<ScalarT> mrpToPrv(const Eigen::Vector3<ScalarT>& mrp, ScalarT localEps = ScalarT(1e-10)) {
    ScalarT norm = mrp.norm();
    if (norm < localEps) {
        return Eigen::Vector3<ScalarT>::Zero();
    }
    return (mrp / norm) * (4.0 * std::atan(norm));
}

/**
 * Translate a MRP vector into a 321 Euler angle vector.
 * @param mrp
 * @return Eigen::Vector3d
 */
template <typename ScalarT>
Eigen::Vector3<ScalarT> mrpToEulerAngles321(const Eigen::Vector3<ScalarT>& mrp) {
    return epToEulerAngles321(mrpToEp(mrp));
}

/**
 * Return the direction cosine matrix corresponding to a principal rotation vector
 * @param prv
 * @return Eigen::Matrix3d
 */
template <typename ScalarT>
Eigen::Matrix3<ScalarT> prvToDcm(const Eigen::Vector3<ScalarT>& prv) {
    ScalarT angle = prv.norm();

    if (angle < Types<ScalarT>::eps) {
        return Eigen::Matrix3<ScalarT>::Identity();
    }

    Eigen::Vector3<ScalarT> u = prv / angle;
    ScalarT c = std::cos(angle);
    ScalarT s = std::sin(angle);
    ScalarT one_minus_c = 1.0 - c;

    Eigen::Matrix3<ScalarT> dcm;
    dcm(0, 0) = u(0) * u(0) * one_minus_c + c;
    dcm(0, 1) = u(0) * u(1) * one_minus_c + u(2) * s;
    dcm(0, 2) = u(0) * u(2) * one_minus_c - u(1) * s;
    dcm(1, 0) = u(1) * u(0) * one_minus_c - u(2) * s;
    dcm(1, 1) = u(1) * u(1) * one_minus_c + c;
    dcm(1, 2) = u(1) * u(2) * one_minus_c + u(0) * s;
    dcm(2, 0) = u(2) * u(0) * one_minus_c + u(1) * s;
    dcm(2, 1) = u(2) * u(1) * one_minus_c - u(0) * s;
    dcm(2, 2) = u(2) * u(2) * one_minus_c + c;

    return dcm;
}

/**
 * Translate a principal rotation vector into an Euler parameter vector.
 * @param prv
 * @return Eigen::Vector4d
 */
template <typename ScalarT>
Eigen::Vector4<ScalarT> prvToEp(const Eigen::Vector3<ScalarT>& prv) {
    ScalarT angle = prv.norm();
    Eigen::Vector<ScalarT, 4> ep;

    if (constexpr ScalarT localEps = ScalarT(1e-12); angle < localEps) {
        ep << ScalarT(1), ScalarT(0), ScalarT(0), ScalarT(0);
        return ep;
    }

    Eigen::Vector<ScalarT, 3> axis = prv / angle;
    ScalarT half_angle = angle / ScalarT(2);

    ep(0) = std::cos(half_angle);
    ep.template tail<3>() = axis * std::sin(half_angle);

    return ep;
}

/**
 * Translate the principal rotation vector into the mrp vector.
 * @param prv
 * @return Eigen::Vector3d
 */
template <typename ScalarT>
Eigen::Vector3<ScalarT> prvToMrp(const Eigen::Vector3<ScalarT>& prv) {
    constexpr ScalarT localEps = ScalarT(1e-12);
    if (ScalarT norm = prv.norm(); norm > localEps) {
        return prv.normalized() * std::tan(norm / ScalarT(4));
    }
    return Eigen::Vector3<ScalarT>::Zero();
}

/**
 * Translate a principal rotation vector into a 321 Euler angle vector.
 * @param prv
 * @return Eigen::Vector3d
 */
template <typename ScalarT>
Eigen::Vector3<ScalarT> prvToEulerAngles321(const Eigen::Vector3<ScalarT>& prv) {
    return epToEulerAngles321(prvToEp(prv));
}

/**
 * Return the direction cosine matrix corresponding to a 321 Euler angle rotation.
 * @param euler321
 * @return Eigen::Matrix3d
 */
template <typename ScalarT>
Eigen::Matrix3<ScalarT> eulerAngles321ToDcm(const Eigen::Vector3<ScalarT>& euler321) {
    ScalarT sin1 = std::sin(euler321(0));
    ScalarT sin2 = std::sin(euler321(1));
    ScalarT sin3 = std::sin(euler321(2));
    ScalarT cos1 = std::cos(euler321(0));
    ScalarT cos2 = std::cos(euler321(1));
    ScalarT cos3 = std::cos(euler321(2));

    Eigen::Matrix3<ScalarT> dcm;
    dcm << cos2 * cos1, cos2 * sin1, -sin2, sin3 * sin2 * cos1 - cos3 * sin1, sin3 * sin2 * sin1 + cos3 * cos1,
        sin3 * cos2, cos3 * sin2 * cos1 + sin3 * sin1, cos3 * sin2 * sin1 - sin3 * cos1, cos3 * cos2;

    return dcm;
}

/**
 * Translate a 321 Euler angle vector into the Euler parameter vector.
 * @param euler321
 * @return Eigen::Vector4d
 */
template <typename ScalarT>
Eigen::Vector4<ScalarT> eulerAngles321ToEp(const Eigen::Vector3<ScalarT>& euler321) {
    ScalarT half0 = euler321(0) / 2.0;
    ScalarT half1 = euler321(1) / 2.0;
    ScalarT half2 = euler321(2) / 2.0;

    ScalarT cos1 = std::cos(half0);
    ScalarT cos2 = std::cos(half1);
    ScalarT cos3 = std::cos(half2);
    ScalarT sin1 = std::sin(half0);
    ScalarT sin2 = std::sin(half1);
    ScalarT sin3 = std::sin(half2);

    Eigen::Vector4<ScalarT> ep;
    ep(0) = cos1 * cos2 * cos3 + sin1 * sin2 * sin3;
    ep(1) = cos1 * cos2 * sin3 - sin1 * sin2 * cos3;
    ep(2) = cos1 * sin2 * cos3 + sin1 * cos2 * sin3;
    ep(3) = sin1 * cos2 * cos3 - cos1 * sin2 * sin3;

    return ep;
}

/**
 * Translate a 321 Euler angle vector into the MRP vector.
 * @param euler321
 * @return Eigen::Vector3d
 */
template <typename ScalarT>
Eigen::Vector3<ScalarT> eulerAngles321ToMrp(const Eigen::Vector3<ScalarT>& euler321) {
    return epToMrp(eulerAngles321ToEp(euler321));
}

/**
 * Translate a 321 Euler angle vector into a principal rotation vector.
 * @param euler321
 * @return Eigen::Vector3d
 */
template <typename ScalarT>
Eigen::Vector3<ScalarT> eulerAngles321ToPrv(const Eigen::Vector3<ScalarT>& euler321) {
    return epToPrv(eulerAngles321ToEp(euler321));
}

/**
 * Provide the Euler parameter vector which corresponds to performing to successive rotations B1 and B2
 * @param ep1 Eigen::Vector4d
 * @param ep2 Eigen::Vector4d
 * @return Eigen::Vector4d
 */
template <typename ScalarT>
Eigen::Vector4<ScalarT> addEp(const Eigen::Vector4<ScalarT>& ep1, const Eigen::Vector4<ScalarT>& ep2) {
    Eigen::Vector4<ScalarT> ep;

    ep(0) = ep2(0) * ep1(0) - ep2(1) * ep1(1) - ep2(2) * ep1(2) - ep2(3) * ep1(3);
    ep(1) = ep2(1) * ep1(0) + ep2(0) * ep1(1) + ep2(3) * ep1(2) - ep2(2) * ep1(3);
    ep(2) = ep2(2) * ep1(0) - ep2(3) * ep1(1) + ep2(0) * ep1(2) + ep2(1) * ep1(3);
    ep(3) = ep2(3) * ep1(0) + ep2(2) * ep1(1) - ep2(1) * ep1(2) + ep2(0) * ep1(3);

    return ep;
}

/**
 * Compute the overall 321 Euler angle vector corresponding to two successive 321) rotations.
 * @param euler3211 Eigen::Vector3d
 * @param euler3212 Eigen::Vector3d
 * @return Eigen::Vector3d
 */
template <typename ScalarT>
Eigen::Vector3<ScalarT> addEulerAngles321(const Eigen::Vector3<ScalarT>& euler3211,
                                          const Eigen::Vector3<ScalarT>& euler3212) {
    return dcmToEulerAngles321((eulerAngles321ToDcm(euler3212) * eulerAngles321ToDcm(euler3211)).eval());
}

/**
 * Provide the principal rotation vector which corresponds to performing successive principal rotations Q1 and Q2.
 * @param prv1 Eigen::Vector3d
 * @param prv2 Eigen::Vector3d
 * @return Eigen::Vector3d
 */
template <typename ScalarT>
Eigen::Vector3<ScalarT> addPrv(const Eigen::Vector3<ScalarT>& prv1, const Eigen::Vector3<ScalarT>& prv2) {
    if (prv1.norm() < ScalarT(1.0E-7) || prv2.norm() < ScalarT(1.0E-7)) {
        return prv1 + prv2;
    }

    ScalarT phi1 = prv1.norm() / 2.0;
    ScalarT phi2 = prv2.norm() / 2.0;

    ScalarT cosPhi1 = std::cos(phi1);
    ScalarT cosPhi2 = std::cos(phi2);
    ScalarT sinPhi1 = std::sin(phi1);
    ScalarT sinPhi2 = std::sin(phi2);

    Eigen::Vector3<ScalarT> unitVector1 = prv1 / prv1.norm();
    Eigen::Vector3<ScalarT> unitVector2 = prv2 / prv2.norm();

    ScalarT dotProduct = unitVector1.dot(unitVector2);
    ScalarT realPart = cosPhi1 * cosPhi2 - sinPhi1 * sinPhi2 * dotProduct;

    assert(std::abs(realPart) < 1.0);

    ScalarT angle = 2.0 * std::acos(realPart);

    if (std::abs(angle) < ScalarT(1.0E-13)) {
        return Eigen::Vector3<ScalarT>::Zero();
    }

    Eigen::Vector3<ScalarT> vectorPart = cosPhi1 * sinPhi2 * unitVector2 + cosPhi2 * sinPhi1 * unitVector1 +
                                         sinPhi1 * sinPhi2 * unitVector1.cross(unitVector2);

    return vectorPart * angle / std::sin(angle / 2.0);
}

/**
 * Provide the mrp parameter vector which corresponds to performing to successive rotations q1 and q2
 * @param mrp1 Eigen::Vector3d
 * @param mrp2 Eigen::Vector3d
 * @return Eigen::Vector3d
 */
template <typename ScalarT>
Eigen::Vector3<ScalarT> addMrp(const Eigen::Vector3<ScalarT>& mrp1, const Eigen::Vector3<ScalarT>& mrp2) {
    Eigen::Vector3<ScalarT> sigma1 = mrp1;
    ScalarT denominator = 1.0 + mrp1.dot(mrp1) * mrp2.dot(mrp2) - 2.0 * mrp1.dot(mrp2);

    if (std::abs(denominator) < ScalarT(0.1)) {
        sigma1 = mrpShadow<ScalarT>(mrp1);  // Shadow set
        denominator = 1.0 + sigma1.dot(sigma1) * mrp2.dot(mrp2) - 2.0 * sigma1.dot(mrp2);
    }

    assert(std::abs(denominator) > Types<ScalarT>::eps);

    Eigen::Vector3<ScalarT> numerator =
        (1.0 - sigma1.dot(sigma1)) * mrp2 + (1.0 - mrp2.dot(mrp2)) * sigma1 - 2.0 * mrp2.cross(sigma1);

    Eigen::Vector3<ScalarT> mrp = numerator / denominator;

    // Map MRP to the inner set
    mrp = mrpSwitch<ScalarT>(mrp, 1.0);

    return mrp;
}

/**
 * Provide the Euler parameter vector which corresponds to relative rotation from B2 to B1.
 * @param ep1
 * @param ep2
 * @return Eigen::Vector4d
 */
template <typename ScalarT>
Eigen::Vector4<ScalarT> subEp(const Eigen::Vector4<ScalarT>& ep1, const Eigen::Vector4<ScalarT>& ep2) {
    Eigen::Vector4<ScalarT> ep;

    ep(0) = ep2(0) * ep1(0) + ep2(1) * ep1(1) + ep2(2) * ep1(2) + ep2(3) * ep1(3);
    ep(1) = -ep2(1) * ep1(0) + ep2(0) * ep1(1) + ep2(3) * ep1(2) - ep2(2) * ep1(3);
    ep(2) = -ep2(2) * ep1(0) - ep2(3) * ep1(1) + ep2(0) * ep1(2) + ep2(1) * ep1(3);
    ep(3) = -ep2(3) * ep1(0) + ep2(2) * ep1(1) - ep2(1) * ep1(2) + ep2(0) * ep1(3);

    return ep;
}

/**
 * Provide the MRP vector which corresponds to relative rotation from Q2 to Q1.
 * @param mrp1
 * @param mrp2
 * @return Eigen::Vector3d
 */
template <typename ScalarT>
Eigen::Vector3<ScalarT> subMrp(const Eigen::Vector3<ScalarT>& mrp1, const Eigen::Vector3<ScalarT>& mrp2) {
    Eigen::Vector3<ScalarT> mrp1Shadow(mrp1);
    ScalarT denominator = ScalarT(1) + mrp2.dot(mrp2) * mrp1.dot(mrp1) + ScalarT(2) * mrp2.dot(mrp1);
    if (std::abs(denominator) < ScalarT(0.1)) {
        mrp1Shadow = mrpShadow(mrp1);  // Shadow set
        denominator = ScalarT(1) + mrp2.dot(mrp2) * mrp1Shadow.dot(mrp1Shadow) + ScalarT(2) * mrp2.dot(mrp1Shadow);
    }
    assert(std::abs(denominator) > Types<ScalarT>::eps);
    Eigen::Vector3<ScalarT> numerator;
    numerator = (ScalarT(1) - mrp2.dot(mrp2)) * mrp1Shadow - (ScalarT(1) - mrp1Shadow.dot(mrp1Shadow)) * mrp2 +
                ScalarT(2) * mrp1Shadow.cross(mrp2);
    Eigen::Vector3<ScalarT> mrp = numerator / denominator;
    /* map mrp to inner set */
    mrp = mrpSwitch(mrp, ScalarT(1));

    return mrp;
}

/**
 * Provide the principal rotation vector which corresponds to relative principal rotation from Q2
 * to Q1.
 * @param prv1
 * @param prv2
 * @return Eigen::Vector3d
 */
template <typename ScalarT>
Eigen::Vector3<ScalarT> subPrv(const Eigen::Vector3<ScalarT>& prv1, const Eigen::Vector3<ScalarT>& prv2) {
    if (constexpr ScalarT threshold = ScalarT(1.0E-7); prv1.norm() < threshold || prv2.norm() < threshold) {
        return prv1 - prv2;
    }

    ScalarT norm1 = prv1.norm();
    ScalarT norm2 = prv2.norm();

    ScalarT cosPhi1 = std::cos(norm1 / ScalarT(2));
    ScalarT cosPhi2 = std::cos(norm2 / ScalarT(2));
    ScalarT sinPhi1 = std::sin(norm1 / ScalarT(2));
    ScalarT sinPhi2 = std::sin(norm2 / ScalarT(2));

    Eigen::Vector3<ScalarT> unitVector1 = prv1 / norm1;
    Eigen::Vector3<ScalarT> unitVector2 = prv2 / norm2;

    ScalarT dotUV = unitVector1.dot(unitVector2);
    ScalarT acosArg = cosPhi1 * cosPhi2 + sinPhi1 * sinPhi2 * dotUV;

    assert(std::abs(acosArg) <= ScalarT(1));

    ScalarT angle = ScalarT(2) * std::acos(acosArg);

    if (std::abs(angle) < ScalarT(1.0E-13)) {
        return Eigen::Vector3<ScalarT>::Zero();
    }

    Eigen::Vector3<ScalarT> prv = cosPhi2 * sinPhi1 * unitVector1 - cosPhi1 * sinPhi2 * unitVector2 +
                                  sinPhi1 * sinPhi2 * unitVector1.cross(unitVector2);

    return prv * angle / std::sin(angle / ScalarT(2));
}

/**
 * Compute the relative 321 Euler angle vector from E1 to E.
 * @param euler3211
 * @param euler3212
 * @return Eigen::Vector3d
 */
template <typename ScalarT>
Eigen::Vector3<ScalarT> subEulerAngles321(const Eigen::Vector3<ScalarT>& euler3211,
                                          const Eigen::Vector3<ScalarT>& euler3212) {
    Eigen::Matrix3<ScalarT> dcm1 = eulerAngles321ToDcm(euler3211);
    Eigen::Matrix3<ScalarT> dcm2 = eulerAngles321ToDcm(euler3212);
    Eigen::Matrix3<ScalarT> dcmDiff = dcm1 * dcm2.transpose();

    return dcmToEulerAngles321(dcmDiff);
}

/**
 * Return the rotation matrix corresponding to a single axis rotation about axis a by the angle theta
 * @param angle
 * @param axis_number
 * @return Eigen::Matrix3d
 */
template <typename ScalarT>
Eigen::Matrix3<ScalarT> rotationMatrix(const ScalarT angle, const int axis_number) {
    assert(axis_number > 0 && axis_number < 4);
    Eigen::Matrix3<ScalarT> dcm;

    if (axis_number == 1) {
        dcm << ScalarT(1), ScalarT(0), ScalarT(0), ScalarT(0), std::cos(angle), std::sin(angle), ScalarT(0),
            -std::sin(angle), std::cos(angle);
    } else if (axis_number == 2) {
        dcm << std::cos(angle), ScalarT(0), -std::sin(angle), ScalarT(0), ScalarT(1), ScalarT(0), std::sin(angle),
            ScalarT(0), std::cos(angle);
    } else {  // axis_number == 3
        dcm << std::cos(angle), std::sin(angle), ScalarT(0), -std::sin(angle), std::cos(angle), ScalarT(0), ScalarT(0),
            ScalarT(0), ScalarT(1);
    }

    return dcm;
}

#endif  // RIGIDBODYKINEMATICS_HPP
