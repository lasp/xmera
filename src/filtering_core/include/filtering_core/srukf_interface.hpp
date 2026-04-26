// SPDX-License-Identifier: ISC
// Copyright (c) 2026, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

#ifndef FILTERING_CORE_SRUKF_INTERFACE_HPP
#define FILTERING_CORE_SRUKF_INTERFACE_HPP

#include <filtering_core/concepts.hpp>
#include <filtering_core/dynamics_model.hpp>

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/QR>

#include <array>
#include <cmath>

namespace filtering {

// Per-filter Spec contract. Any class type X satisfies SrukfSpec if it
// declares:
//   - using State = ...;                           (FilterState)
//   - using ProcessNoiseCov = Eigen::Matrix<...>;  (N x N)
// Plus, at the call site, a dynamics function compatible with the
// Dynamics<D, State> concept and one or more Measurement types compatible
// with Measurement<M, State>.
template<class Spec>
concept SrukfSpec = FilterState<typename Spec::State> && requires {
    typename Spec::ProcessNoiseCov;
};

// Square-root UKF storage. Plain data — no behavior. Pure functions in the
// `srukf` namespace operate on it; `SrukfInterface<Spec>` (defined later)
// is the stateful façade that holds an instance.
template<FilterState State>
struct SrukfStorage {
    static constexpr int N = State::size;
    static constexpr int numSigmaPoints = 2 * N + 1;

    using StateMatrix = Eigen::Matrix<double, N, N>;
    using SigmaWeights = Eigen::Vector<double, numSigmaPoints>;

    // Filter mean and square-root covariance.
    State        mean;
    StateMatrix  sqrtCovar;       // sBar — lower triangular such that covar = sBar * sBar^T

    // Process noise (square root) — fixed across reset()s by setProcessNoise().
    StateMatrix  cholProcessNoise;

    // Initial conditions captured on the way to reset().
    State        meanInitial;
    StateMatrix  covarInitial;
    StateMatrix  processNoise;

    // Tunables.
    double alpha          = 0;
    double beta           = 0;
    double measNoiseScale = 1;
    double unitConversion = 1;

    // Derived from tunables (computed in reset()).
    double lambda = 0;
    double eta    = 0;

    // Sigma-point weights and propagated sigma points carried between
    // predict() and update().
    SigmaWeights              wM = SigmaWeights::Zero();
    SigmaWeights              wC = SigmaWeights::Zero();
    std::array<State, numSigmaPoints> sigmaPoints = {};
    State                     xBar;
};

namespace srukf {

// ------ Numerical helpers (file-private) ------------------------------------

namespace detail {

// QR decomposition returning only the top-left R block (transposed). The
// SRUKF needs only R, not Q, and only the leading n×n block where n =
// input.rows(). Input is expected to have more cols than rows.
inline Eigen::MatrixXd qrDecompositionJustR(Eigen::MatrixXd const& input) {
    Eigen::HouseholderQR<Eigen::MatrixXd> qrDecomposition(input.transpose());
    Eigen::MatrixXd const Q = qrDecomposition.householderQ();
    Eigen::MatrixXd const R = Q.transpose() * input.transpose();
    Eigen::MatrixXd RTilde = R.block(0, 0, input.rows(), input.rows());

    // Zero anything that should be zero to avoid numeric drift.
    for (int i = 0; i < RTilde.rows(); ++i) {
        for (int j = 0; j < i; ++j) {
            RTilde(i, j) = 0;
        }
    }

    return RTilde.transpose();
}

inline Eigen::MatrixXd choleskyDecomposition(Eigen::MatrixXd const& input) {
    Eigen::LLT<Eigen::MatrixXd> chol(input);
    return chol.matrixL();
}

// Cholesky rank-1 up/down date. P = S*S^T  ->  P' = P + sign*|coef|*v*v^T;
// returns Cholesky-factor of P' via QR decomposition of the explicitly
// reformed P'.
inline Eigen::MatrixXd choleskyUpDownDate(
    Eigen::MatrixXd const& sqrtP,
    Eigen::VectorXd const& v,
    double coefficient
) {
    Eigen::MatrixXd P = sqrtP * sqrtP.transpose();
    int const sign = (coefficient > 0) ? 1 : -1;
    P += sign * std::abs(coefficient) * v * v.transpose();

    Eigen::MatrixXd const A = choleskyDecomposition(P);
    return qrDecompositionJustR(A);
}

// Generic forward substitution: solve L*x = b, L lower triangular.
inline Eigen::MatrixXd forwardSubstitution(Eigen::MatrixXd const& L, Eigen::MatrixXd const& b) {
    Eigen::MatrixXd x = Eigen::MatrixXd::Zero(b.rows(), b.cols());
    for (int col = 0; col < b.cols(); ++col) {
        Eigen::VectorXd xCol = Eigen::VectorXd::Zero(b.rows());
        for (int i = 0; i < L.rows(); ++i) {
            xCol(i) = b(i, col);
            for (int j = 0; j < i; ++j) {
                xCol(i) -= L(i, j) * xCol(j);
            }
            xCol(i) /= L(i, i);
        }
        x.col(col) = xCol;
    }
    return x;
}

// Generic back substitution: solve U*x = b, U upper triangular.
inline Eigen::MatrixXd backSubstitution(Eigen::MatrixXd const& U, Eigen::MatrixXd const& b) {
    Eigen::MatrixXd x = Eigen::MatrixXd::Zero(b.rows(), b.cols());
    for (int col = 0; col < b.cols(); ++col) {
        Eigen::VectorXd xCol = Eigen::VectorXd::Zero(b.rows());
        for (long i = U.rows() - 1; i >= 0; --i) {
            xCol(i) = b(i, col);
            for (long j = i + 1; j < U.rows(); ++j) {
                xCol(i) -= U(i, j) * xCol(j);
            }
            xCol(i) /= U(i, i);
        }
        x.col(col) = xCol;
    }
    return x;
}

}  // namespace detail

// ------ Functional core: reset / predict / update ---------------------------

// Recompute derived tunables, sigma weights, and the Cholesky factors of the
// initial covariance and process noise. Returns the updated storage.
template<SrukfSpec Spec>
SrukfStorage<typename Spec::State> reset(SrukfStorage<typename Spec::State> s) {
    using Storage = SrukfStorage<typename Spec::State>;
    constexpr int N = Storage::N;

    s.mean = s.meanInitial.scale(s.unitConversion);

    auto const initialCovar = s.unitConversion * s.unitConversion * s.covarInitial;
    s.sqrtCovar = detail::choleskyDecomposition(initialCovar);

    s.cholProcessNoise =
        detail::choleskyDecomposition(s.unitConversion * s.unitConversion * s.processNoise);

    s.lambda = static_cast<double>(N) * (s.alpha * s.alpha - 1.0);
    s.eta    = std::sqrt(static_cast<double>(N) + s.lambda);

    s.wM(0) = s.lambda / (static_cast<double>(N) + s.lambda);
    s.wC(0) = s.lambda / (static_cast<double>(N) + s.lambda) + (1.0 - s.alpha * s.alpha + s.beta);
    for (int i = 1; i < Storage::numSigmaPoints; ++i) {
        s.wM(i) = 1.0 / (2.0 * (static_cast<double>(N) + s.lambda));
        s.wC(i) = s.wM(i);
    }

    return s;
}

// Time update. Propagates 2N+1 sigma points through Spec::dynamics, recovers
// the mean (`xBar`) and sqrt-covariance (`sqrtCovar`), and stores the
// propagated sigma points for use by the next update().
template<SrukfSpec Spec, Dynamics<typename Spec::State> D>
SrukfStorage<typename Spec::State> predict(
    SrukfStorage<typename Spec::State> s,
    D const& dynamics,
    double dt
) {
    using Storage = SrukfStorage<typename Spec::State>;
    constexpr int N = Storage::N;
    std::array<double, 2> const interval = {0, dt};

    // sigmaPoints[0] is the propagated mean.
    s.sigmaPoints[0] = filtering::propagate(dynamics, s.mean, interval, dt);
    s.xBar = s.sigmaPoints[0].scale(s.wM(0));

    // The remaining 2N points are mean ± eta * column-of-sqrtCovar.
    for (int i = 1; i <= N; ++i) {
        typename Spec::State::Storage const offset = s.eta * s.sqrtCovar.col(i - 1);
        // Build offset states by constructing from raw storage, then
        // propagating through the dynamics. (We don't assume State has an
        // addVector(raw_storage) method.)
        typename Spec::State const plus  = typename Spec::State(s.mean.raw() + offset);
        typename Spec::State const minus = typename Spec::State(s.mean.raw() - offset);
        s.sigmaPoints[i]     = filtering::propagate(dynamics, plus,  interval, dt);
        s.sigmaPoints[i + N] = filtering::propagate(dynamics, minus, interval, dt);
    }

    // xBar = sum_i wM(i) * sigmaPoints[i]
    for (int i = 1; i < Storage::numSigmaPoints; ++i) {
        s.xBar = s.xBar.add(s.sigmaPoints[i].scale(s.wM(i)));
    }

    // Build the QR-decomposition input matrix A from the (N) deviations of
    // each sigma point off xBar plus the Cholesky of the process noise.
    Eigen::MatrixXd A(N, 3 * N);
    for (int i = 1; i < Storage::numSigmaPoints; ++i) {
        A.col(i - 1) =
            std::sqrt(s.wC(i)) * (s.sigmaPoints[i].raw() - s.xBar.raw());
    }
    A.block(0, Storage::numSigmaPoints - 1, N, N) = s.cholProcessNoise;

    Eigen::MatrixXd const sBar = detail::qrDecompositionJustR(A);

    // Cholesky update with the 0th sigma-point deviation, weighted by wC(0).
    Eigen::VectorXd const xError = s.sigmaPoints[0].raw() - s.xBar.raw();
    s.sqrtCovar = detail::choleskyUpDownDate(sBar, xError, s.wC(0));
    s.mean      = s.sigmaPoints[0];

    return s;
}

// Result of an update: posterior storage plus pre- and post-fit residuals
// for diagnostics.
template<class State, class M>
struct UpdateResult {
    SrukfStorage<State>             posterior;
    Eigen::Vector<double, M::size>  preFit;
    Eigen::Vector<double, M::size>  postFit;
};

// Measurement update. `m` must satisfy Measurement<M, State> from
// concepts.hpp.
template<SrukfSpec Spec, Measurement<typename Spec::State> M>
UpdateResult<typename Spec::State, M> update(
    SrukfStorage<typename Spec::State> s,
    M const& measurement
) {
    using Storage = SrukfStorage<typename Spec::State>;
    constexpr int N = Storage::N;
    int const numSigma = Storage::numSigmaPoints;

    auto const observation  = measurement.observation();
    int  const M_size       = static_cast<int>(observation.size());

    // Pre-fit residuals (using already-propagated sigma points from predict).
    Eigen::MatrixXd yMeasPre(M_size, numSigma);
    for (int j = 0; j < numSigma; ++j) {
        yMeasPre.col(j) = measurement.model(s.sigmaPoints[j]);
    }
    Eigen::VectorXd yBarPre = Eigen::VectorXd::Zero(M_size);
    for (int i = 0; i < numSigma; ++i) {
        yBarPre += s.wM(i) * yMeasPre.col(i);
    }
    Eigen::Vector<double, M::size> const preFit = measurement.subtract(observation, yBarPre);

    // Cholesky factor of measurement noise.
    Eigen::MatrixXd const cholMeasNoise = detail::choleskyDecomposition(measurement.noise());

    // (Re-using yMeasPre / yBarPre — math is identical to recomputing.)
    Eigen::MatrixXd const& yMeas = yMeasPre;
    Eigen::VectorXd const& yBar  = yBarPre;

    // QR-decomposition input matrix A for sy = sqrt-covariance of innovation.
    Eigen::MatrixXd A(M_size, 2 * N + M_size);
    for (int i = 1; i < numSigma; ++i) {
        A.col(i - 1) = std::sqrt(s.wC(1)) * (yMeas.col(i) - yBar);
    }
    A.block(0, numSigma - 1, M_size, M_size) = cholMeasNoise;

    Eigen::MatrixXd sy = detail::qrDecompositionJustR(A);

    Eigen::VectorXd const yError0 = yMeas.col(0) - yBar;
    sy = detail::choleskyUpDownDate(sy, yError0, s.wC(0));

    // Cross-covariance Pxy = sum_i wC(i) * (xi - xBar)(yi - yBar)^T.
    Eigen::MatrixXd pXY = Eigen::MatrixXd::Zero(N, M_size);
    for (int i = 0; i < numSigma; ++i) {
        Eigen::VectorXd const xError = s.sigmaPoints[i].raw() - s.xBar.raw();
        Eigen::VectorXd const yError = yMeas.col(i) - yBar;
        pXY += s.wC(i) * xError * yError.transpose();
    }

    // K = Pxy * (sy^T * sy)^-1, computed via two triangular solves.
    Eigen::MatrixXd const stKt = detail::forwardSubstitution(sy, pXY.transpose());
    Eigen::MatrixXd const kMat = detail::backSubstitution(sy.transpose(), stKt).transpose();

    // State update: mean = xBar + K * (observation - yBar).
    Eigen::VectorXd const innovation = measurement.subtract(observation, yBar);
    typename Spec::State updatedMean = typename Spec::State(s.xBar.raw() + kMat * innovation);
    s.mean = updatedMean;

    // Covariance update: U = K * sy; downdate sqrtCovar by each column of U.
    Eigen::MatrixXd const Umat = kMat * sy;
    for (int i = 0; i < Umat.cols(); ++i) {
        s.sqrtCovar = detail::choleskyUpDownDate(s.sqrtCovar, Umat.col(i), -1);
    }

    // Post-fit residuals.
    Eigen::Vector<double, M::size> const postFit =
        measurement.subtract(observation, measurement.model(s.mean));

    return { .posterior = s, .preFit = preFit, .postFit = postFit };
}

}  // namespace srukf

}  // namespace filtering

#endif
