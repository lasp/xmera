// SPDX-License-Identifier: ISC

#ifndef BASILISK_STATICFILTERTRAITS_HPP
#define BASILISK_STATICFILTERTRAITS_HPP

#include <Eigen/Core>

/**
 * @brief Common compile-time aliases for state and measurement quantities.
 *
 * The FilterTraits template encapsulates the scalar type and state dimension that the filter
 * operates on. Measurement-related aliases are provided through the nested MeasurementTraits
 * struct, allowing each sensor model to declare its own compile-time dimension while still
 * sharing the state-centric definitions.
 */
template <typename ScalarT, int StateDimT>
struct FilterTraits {
    static_assert(StateDimT > 0, "State dimension must be strictly positive.");

    using Scalar = ScalarT;
    static constexpr int state_dim = StateDimT;

    using StateVector = Eigen::Matrix<Scalar, state_dim, 1>;
    using StateMatrix = Eigen::Matrix<Scalar, state_dim, state_dim>;
    using ProcessNoiseMatrix = StateMatrix;

    /**
     * @brief Measurement-specific aliases bound to the owning state dimension.
     *
     * @tparam MeasDimT Compile-time dimension of the measurement vector.
     */
    template <int MeasDimT>
    struct MeasurementTraits {
        static_assert(MeasDimT > 0, "Measurement dimension must be strictly positive.");

        static constexpr int meas_dim = MeasDimT;

        using Vector = Eigen::Matrix<Scalar, meas_dim, 1>;
        using Covariance = Eigen::Matrix<Scalar, meas_dim, meas_dim>;
        using Jacobian = Eigen::Matrix<Scalar, meas_dim, state_dim>;
        using KalmanGain = Eigen::Matrix<Scalar, state_dim, meas_dim>;
        using CrossCovariance = Eigen::Matrix<Scalar, state_dim, meas_dim>;
    };

    static StateVector zeroState() { return StateVector::Zero(); }

    static StateMatrix identityCovariance() { return StateMatrix::Identity(); }
};

#endif  // BASILISK_STATICFILTERTRAITS_HPP
