// SPDX-License-Identifier: ISC
// Copyright (c) 2026, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

#ifndef FILTERING_CORE_SRUKF_HPP
#define FILTERING_CORE_SRUKF_HPP

#include <filteringCore/concepts.hpp>
#include <filteringCore/dynamicsModel.hpp>

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/QR>

#include <array>
#include <math.h>

namespace filtering {

/*! Square-root UKF (van der Merwe & Wan, ICASSP 2001). Owns all filter state
 *  plus a settable dynamics functor; reset / timeUpdate / measurementUpdate
 *  mutate state in place. */
template<FilterState State, Dynamics<State> Dyn>
class SRuKF {
   public:
    using Covariance   = Eigen::Matrix<double, State::size, State::size>;
    using SigmaWeights = Eigen::Vector<double, 2 * State::size + 1>;

    /*! Pre- and post-fit residuals returned from measurementUpdate(). */
    template<class M>
    struct UpdateResult {
        Eigen::Vector<double, M::size> preFit;
        Eigen::Vector<double, M::size> postFit;
    };

    /*! QR decomposition returning only the leading Rows×Rows block of R
     *  (transposed). Input must be wider than tall.
     *  @return Rows×Rows lower-triangular factor
     *  @param input [-] Rows×Cols matrix (Cols >= Rows) */
    template<int Rows, int Cols>
    static Eigen::Matrix<double, Rows, Rows> qrDecompositionJustR(
        Eigen::Matrix<double, Rows, Cols> const& input
    ) {
        Eigen::Matrix<double, Cols, Rows> const inputT = input.transpose();
        Eigen::HouseholderQR<Eigen::Matrix<double, Cols, Rows>> qr(inputT);
        Eigen::Matrix<double, Cols, Cols> const Q = qr.householderQ();
        Eigen::Matrix<double, Cols, Rows> const R = Q.transpose() * inputT;
        Eigen::Matrix<double, Rows, Rows> RTilde = R.template block<Rows, Rows>(0, 0);

        for (int i = 0; i < Rows; ++i) {
            for (int j = 0; j < i; ++j) {
                RTilde(i, j) = 0;
            }
        }

        return RTilde.transpose();
    }

    /*! Cholesky decomposition.
     *  @return lower-triangular L with input = L * L^T
     *  @param input [-] symmetric positive-definite matrix */
    template<int Size>
    static Eigen::Matrix<double, Size, Size> choleskyDecomposition(
        Eigen::Matrix<double, Size, Size> const& input
    ) {
        Eigen::LLT<Eigen::Matrix<double, Size, Size>> chol(input);
        return chol.matrixL();
    }

    /*! Check whether a square matrix is symmetric positive semi-definite.
     *  Allows a tiny negative tolerance on the smallest eigenvalue to absorb
     *  floating-point roundoff from the eigen solver.
     *  @return true iff symmetric and smallest eigenvalue >= -1e-12
     *  @param input [-] square matrix to test */
    template<int Size>
    static bool isPositiveSemiDefinite(Eigen::Matrix<double, Size, Size> const& input) {
        if (!input.isApprox(input.transpose())) return false;
        Eigen::SelfAdjointEigenSolver<Eigen::Matrix<double, Size, Size>> solver(input);
        if (solver.info() != Eigen::Success) return false;
        return solver.eigenvalues().minCoeff() >= -1e-12;
    }

    /*! Rank-1 Cholesky up/down date: returns chol(S*S^T + sign(coef)*|coef|*v*v^T).
     *  @return updated Cholesky factor
     *  @param sqrtP       [-] current Cholesky factor S
     *  @param v           [-] rank-1 update vector
     *  @param coefficient [-] signed weight (sign determines up vs down date) */
    template<int Size>
    static Eigen::Matrix<double, Size, Size> choleskyUpDownDate(
        Eigen::Matrix<double, Size, Size> const& sqrtP,
        Eigen::Vector<double, Size> const& v,
        double coefficient
    ) {
        Eigen::Matrix<double, Size, Size> P = sqrtP * sqrtP.transpose();
        int const sign = (coefficient > 0) ? 1 : -1;
        P += sign * fabs(coefficient) * v * v.transpose();

        Eigen::Matrix<double, Size, Size> const A = choleskyDecomposition<Size>(P);
        return qrDecompositionJustR<Size, Size>(A);
    }

    /*! Forward substitution: solve L*x = b with L lower triangular.
     *  @return x
     *  @param L [-] Rows×Rows lower-triangular matrix
     *  @param b [-] Rows×Cols RHS */
    template<int Rows, int Cols>
    static Eigen::Matrix<double, Rows, Cols> forwardSubstitution(
        Eigen::Matrix<double, Rows, Rows> const& L,
        Eigen::Matrix<double, Rows, Cols> const& b
    ) {
        Eigen::Matrix<double, Rows, Cols> x = Eigen::Matrix<double, Rows, Cols>::Zero();
        for (int col = 0; col < Cols; ++col) {
            for (int i = 0; i < Rows; ++i) {
                x(i, col) = b(i, col);
                for (int j = 0; j < i; ++j) {
                    x(i, col) -= L(i, j) * x(j, col);
                }
                x(i, col) /= L(i, i);
            }
        }
        return x;
    }

    /*! Back substitution: solve U*x = b with U upper triangular.
     *  @return x
     *  @param U [-] Rows×Rows upper-triangular matrix
     *  @param b [-] Rows×Cols RHS */
    template<int Rows, int Cols>
    static Eigen::Matrix<double, Rows, Cols> backSubstitution(
        Eigen::Matrix<double, Rows, Rows> const& U,
        Eigen::Matrix<double, Rows, Cols> const& b
    ) {
        Eigen::Matrix<double, Rows, Cols> x = Eigen::Matrix<double, Rows, Cols>::Zero();
        for (int col = 0; col < Cols; ++col) {
            for (int i = Rows - 1; i >= 0; --i) {
                x(i, col) = b(i, col);
                for (int j = i + 1; j < Rows; ++j) {
                    x(i, col) -= U(i, j) * x(j, col);
                }
                x(i, col) /= U(i, i);
            }
        }
        return x;
    }

    void setAlpha(double newAlpha)                                       { this->alpha             = newAlpha; }
    void setBeta(double newBeta)                                         { this->beta              = newBeta; }
    void setProcessNoise(Covariance const& newProcessNoise)              { this->processNoise      = newProcessNoise; }
    void setInitialState(State const& newInitialState)                   { this->stateInitial      = newInitialState; }
    void setInitialCovariance(Covariance const& newInitialCovariance)    { this->covarianceInitial = newInitialCovariance; }

    double getAlpha() const                                              { return this->alpha; }
    double getBeta() const                                               { return this->beta;  }

    Dyn dynamics;  //!< concrete dynamics functor; set by the owning algorithm

    // ---- Validity checks ----------------------------------------------------

    /*! @return true iff alpha is in [0, 1] */
    bool alphaIsValid() const { return this->alpha >= 0.0 && this->alpha <= 1.0; }

    /*! @return true iff beta is in [0, 2] */
    bool betaIsValid()  const { return this->beta  >= 0.0 && this->beta  <= 2.0; }

    /*! @return true iff the configured initial covariance is symmetric PSD */
    bool initialCovarianceIsValid() const { return isPositiveSemiDefinite(this->covarianceInitial); }

    /*! @return true iff the configured process noise is symmetric PSD */
    bool processNoiseIsValid()      const { return isPositiveSemiDefinite(this->processNoise); }

    /*! Initialize from initial conditions; seeds the rolling
     *  last-measurement state and derives lambda, eta, sigma weights. */
    void reset() {
        constexpr int N              = State::size;
        constexpr int numSigmaPoints = 2 * N + 1;
        constexpr double Ndouble = static_cast<double>(N);

        this->state      = this->stateInitial;
        this->covariance = this->covarianceInitial;
        this->sqrtCovar  = choleskyDecomposition<N>(this->covarianceInitial);

        this->stateLastMeasurement      = this->state;
        this->covarianceLastMeasurement = this->covariance;
        this->sqrtCovarLastMeasurement  = this->sqrtCovar;

        this->cholProcessNoise = choleskyDecomposition<N>(this->processNoise);

        this->lambda = Ndouble * (this->alpha * this->alpha - 1.0);
        this->eta    = sqrt(Ndouble + this->lambda);

        this->wM(0) = this->lambda / (Ndouble + this->lambda);
        this->wC(0) = this->lambda / (Ndouble + this->lambda)
                    + (1.0 - this->alpha * this->alpha + this->beta);
        for (int i = 1; i < numSigmaPoints; ++i) {
            this->wM(i) = 1.0 / (2.0 * (Ndouble + this->lambda));
            this->wC(i) = this->wM(i);
        }
    }

    /*! Rewind to the last-measurement state and propagate by dt. dt == 0
     *  short-circuits: sigma points are populated around the anchor for the
     *  next measurementUpdate, no dynamics evolution is applied.
     *  @param dt [s] elapsed time since the last measurement */
    void timeUpdate(const double dt) {
        constexpr int N              = State::size;
        constexpr int numSigmaPoints = 2 * N + 1;

        this->state      = this->stateLastMeasurement;
        this->covariance = this->covarianceLastMeasurement;
        this->sqrtCovar  = this->sqrtCovarLastMeasurement;

        if (dt == 0) {
            this->sigmaPoints[0] = this->state;
            for (int i = 1; i <= N; ++i) {
                typename State::Storage const offset = this->eta * this->sqrtCovar.col(i - 1);
                this->sigmaPoints[i]     = State(this->state.raw() + offset);
                this->sigmaPoints[i + N] = State(this->state.raw() - offset);
            }
            this->xBar = this->state;
        }
        else {
            std::array<double, 2> const interval = {0, dt};

            this->sigmaPoints[0] = filtering::propagate(this->dynamics, this->state, interval);
            this->xBar = this->sigmaPoints[0].scale(this->wM(0));

            for (int i = 1; i <= N; ++i) {
                typename State::Storage const offset = this->eta * this->sqrtCovar.col(i - 1);
                State const plus  = State(this->state.raw() + offset);
                State const minus = State(this->state.raw() - offset);
                this->sigmaPoints[i]     = filtering::propagate(this->dynamics, plus,  interval);
                this->sigmaPoints[i + N] = filtering::propagate(this->dynamics, minus, interval);
            }

            for (int i = 1; i < numSigmaPoints; ++i) {
                this->xBar = this->xBar.add(this->sigmaPoints[i].scale(this->wM(i)));
            }

            Eigen::Matrix<double, N, 3 * N> A;
            for (int i = 1; i < numSigmaPoints; ++i) {
                A.col(i - 1) = sqrt(this->wC(i)) * (this->sigmaPoints[i].raw() - this->xBar.raw());
            }
            A.template block<N, N>(0, numSigmaPoints - 1) = this->cholProcessNoise;

            Eigen::Matrix<double, N, N> const sBar =
                qrDecompositionJustR<N, 3 * N>(A);

            Eigen::Vector<double, N> const xError = this->sigmaPoints[0].raw() - this->xBar.raw();
            this->sqrtCovar  = choleskyUpDownDate<N>(sBar, xError, this->wC(0));
            this->covariance = this->sqrtCovar * this->sqrtCovar.transpose();
            this->state      = this->sigmaPoints[0];
        }
    }

    /*! Fold a single measurement using the sigma points populated by the
     *  previous timeUpdate(). Rolls the last-measurement state forward to the
     *  posterior.
     *  @return pre- and post-fit residuals
     *  @param measurement [-] satisfies Measurement<M, State> */
    template<Measurement<State> M>
    UpdateResult<M> measurementUpdate(M const& measurement) {
        constexpr int N        = State::size;
        constexpr int numSigma = 2 * N + 1;
        constexpr int MSize    = M::size;

        auto const observation = measurement.observation();

        Eigen::Matrix<double, MSize, numSigma> yMeasPre;
        for (int j = 0; j < numSigma; ++j) {
            yMeasPre.col(j) = measurement.model(this->sigmaPoints[j]);
        }
        Eigen::Vector<double, MSize> yBarPre = Eigen::Vector<double, MSize>::Zero();
        for (int i = 0; i < numSigma; ++i) {
            yBarPre += this->wM(i) * yMeasPre.col(i);
        }
        Eigen::Vector<double, MSize> const preFit = measurement.subtract(observation, yBarPre);

        Eigen::Matrix<double, MSize, MSize> const cholMeasNoise =
            choleskyDecomposition<MSize>(measurement.noise());

        Eigen::Matrix<double, MSize, 2 * N + MSize> A;
        for (int i = 1; i < numSigma; ++i) {
            A.col(i - 1) = sqrt(this->wC(1)) * (yMeasPre.col(i) - yBarPre);
        }
        A.template block<MSize, MSize>(0, numSigma - 1) = cholMeasNoise;

        Eigen::Matrix<double, MSize, MSize> sy =
            qrDecompositionJustR<MSize, 2 * N + MSize>(A);

        Eigen::Vector<double, MSize> const yError0 = yMeasPre.col(0) - yBarPre;
        sy = choleskyUpDownDate<MSize>(sy, yError0, this->wC(0));

        Eigen::Matrix<double, N, MSize> pXY = Eigen::Matrix<double, N, MSize>::Zero();
        for (int i = 0; i < numSigma; ++i) {
            Eigen::Vector<double, N>     const xError = this->sigmaPoints[i].raw() - this->xBar.raw();
            Eigen::Vector<double, MSize> const yError = yMeasPre.col(i) - yBarPre;
            pXY += this->wC(i) * xError * yError.transpose();
        }

        Eigen::Matrix<double, MSize, N> const pXYT = pXY.transpose();
        Eigen::Matrix<double, MSize, N> const stKt =
            forwardSubstitution<MSize, N>(sy, pXYT);
        Eigen::Matrix<double, MSize, MSize> const syT = sy.transpose();
        Eigen::Matrix<double, N, MSize> const kMat =
            backSubstitution<MSize, N>(syT, stKt).transpose();

        Eigen::Vector<double, MSize> const innovation = measurement.subtract(observation, yBarPre);
        this->state = State(this->xBar.raw() + kMat * innovation);

        Eigen::Matrix<double, N, MSize> const Umat = kMat * sy;
        for (int i = 0; i < MSize; ++i) {
            Eigen::Vector<double, N> const col = Umat.col(i);
            this->sqrtCovar = choleskyUpDownDate<N>(this->sqrtCovar, col, -1.0);
        }
        this->covariance = this->sqrtCovar * this->sqrtCovar.transpose();

        this->stateLastMeasurement      = this->state;
        this->covarianceLastMeasurement = this->covariance;
        this->sqrtCovarLastMeasurement  = this->sqrtCovar;

        Eigen::Vector<double, MSize> const postFit =
            measurement.subtract(observation, measurement.model(this->state));

        return { .preFit = preFit, .postFit = postFit };
    }

    /*! @return current filter state */
    State      getState()                  const { return this->state; }
    /*! @return last-measurement state (post-last-measurement) */
    State      getStateAtLastMeasurement() const { return this->stateLastMeasurement; }
    /*! @return current full covariance */
    Covariance getCovariance()             const { return this->covariance; }

    /*! Overwrite the working state.
     *  @param newState [-] new working state */
    void setState(State const& newState)                                 { this->state = newState; }
    /*! Overwrite the last-measurement state
     *  @param newStateLastMeas [-] new last-measurement state */
    void setStateLastMeasurement(State const& newStateLastMeas)          { this->stateLastMeasurement = newStateLastMeas; }

   private:
    /*!  Working state at the current call time. */
    State       state;
    Covariance  covariance;
    Covariance  sqrtCovar;

    /*!  Process noise and Cholesky decomposition of process noise. */
    Covariance  processNoise;
    Covariance  cholProcessNoise;

    /*!  Initial conditions */
    State       stateInitial;
    Covariance  covarianceInitial;

    /*!  Rolling last-measurement state. */
    State       stateLastMeasurement;
    Covariance  covarianceLastMeasurement;
    Covariance  sqrtCovarLastMeasurement;

    /*!  Filter parameters.  */
    double alpha  = 0;
    double beta   = 0;
    double lambda = 0;
    double eta    = 0;

    /*!  Sigma weights, latest propagated sigma points, and weighted mean. */
    SigmaWeights                           wM = SigmaWeights::Zero();
    SigmaWeights                           wC = SigmaWeights::Zero();
    std::array<State, 2 * State::size + 1> sigmaPoints = {};
    State                                  xBar;
};

}  // namespace filtering

#endif
