// SPDX-License-Identifier: ISC

// Created by Patrick Kenneally on 5/24/25.
//

#ifndef BASILISK_SRUNSCENTEDKALMANFILTER_H
#define BASILISK_SRUNSCENTEDKALMANFILTER_H

#include "KalmanFilterBase.hpp"
#include "IMeasurement.hpp"
#include "State.hpp"

#include <Eigen/Dense>
#include <functional>

template <int StateDim>
class SRUnscentedKalmanFilter : public KalmanFilterBase<StateDim, SRUnscentedKalmanFilter<StateDim>> {
   public:
    using StateType = State<StateDim>;
    static constexpr int NumSigma = 2 * StateDim + 1;
    using Vector = Eigen::Matrix<double, StateDim, 1>;
    using Matrix = Eigen::Matrix<double, StateDim, StateDim>;
    using SigmaMatrix = Eigen::Matrix<double, StateDim, NumSigma>;

    SRUnscentedKalmanFilter();

    void predictImpl(double currentSimNanos) {
        SigmaMatrix sigmaPoints;

        // 1. Square root of P
        Eigen::LLT<Matrix> llt(state_.P);
        Matrix S = llt.matrixL();  // lower-triangular S such that P = S S^T

        // 2. Generate sigma points
        sigmaPoints.col(0) = state_.x;
        for (int i = 0; i < StateDim; ++i) {
            sigmaPoints.col(i + 1) = state_.x + eta * S.col(i);
            sigmaPoints.col(i + 1 + StateDim) = state_.x - eta * S.col(i);
        }

        // 3. Propagate sigma points through nonlinear model
        for (int i = 0; i < NumSigma; ++i) sigmaPoints.col(i) = fx(sigmaPoints.col(i));

        // 4. Predicted mean
        Vector x_pred = Vector::Zero();
        Eigen::Matrix<double, NumSigma, 1> w_m;
        Eigen::Matrix<double, NumSigma, 1> w_c;

        w_m(0) = lambda / (StateDim + lambda);
        w_c(0) = w_m(0) + (1 - alpha * alpha + beta);
        for (int i = 1; i < NumSigma; ++i) w_m(i) = w_c(i) = 1.0 / (2 * (StateDim + lambda));

        for (int i = 0; i < NumSigma; ++i) x_pred += w_m(i) * sigmaPoints.col(i);

        // 5. Centered and weighted sigma deviations (excluding 0)
        Eigen::Matrix<double, StateDim, NumSigma - 1> Y;
        for (int i = 1; i < NumSigma; ++i) Y.col(i - 1) = std::sqrt(w_c(i)) * (sigmaPoints.col(i) - x_pred);

        // 6. QR decomposition of Y
        Eigen::HouseholderQR<Eigen::Matrix<double, StateDim, NumSigma - 1>> qr(Y);
        Matrix S_pred = qr.householderQ() * Matrix::Identity();

        // 7. Apply rank-one update for 0-th sigma point
        Eigen::Matrix<double, StateDim, 1> y0 = sigmaPoints.col(0) - x_pred;
        S_pred = cholupdate<StateDim>(S_pred, std::sqrt(std::abs(w_c(0))) * y0, w_c(0) > 0);

        // Store result
        state_.x = x_pred;
        state_.P = S_pred * S_pred.transpose();  // or keep just S_pred for future sqrt usage
    }

    // template <typename Meas>
    void updateImpl(typename Meas::Measurement& z) {
        static constexpr int ZDim = Meas::ZDim;
        using ZVec = typename Meas::Measurement;
        using ZMat = typename Meas::NoiseCovSqrt;
        using ZSigmaMat = Eigen::Matrix<double, ZDim, NumSigma>;
        using XVec = typename StateType::Vector;
        using XMat = typename StateType::Covariance;

        Eigen::Matrix<double, StateDim, NumSigma> sigmaPoints;

        // Compute square root of covariance
        Eigen::LLT<XMat> llt(state_.P);
        Eigen::Matrix<double, StateDim, StateDim> S_x = llt.matrixL();

        // Generate sigma points
        sigmaPoints.col(0) = state_.x;
        for (int i = 0; i < StateDim; ++i) {
            sigmaPoints.col(i + 1) = state_.x + eta * S_x.col(i);
            sigmaPoints.col(i + 1 + StateDim) = state_.x - eta * S_x.col(i);
        }

        // Propagate sigma points into measurement space
        ZSigmaMat Zsigmas;
        for (int i = 0; i < NumSigma; ++i) Zsigmas.col(i) = Meas::h(sigmaPoints.col(i));

        // Compute weights
        Eigen::Matrix<double, NumSigma, 1> w_m;
        Eigen::Matrix<double, NumSigma, 1> w_c;
        w_m(0) = lambda / (StateDim + lambda);
        w_c(0) = w_m(0) + (1 - alpha * alpha + beta);
        for (int i = 1; i < NumSigma; ++i) w_m(i) = w_c(i) = 1.0 / (2 * (StateDim + lambda));

        // Predicted measurement mean
        ZVec z_pred = ZVec::Zero();
        for (int i = 0; i < NumSigma; ++i) z_pred += w_m(i) * Zsigmas.col(i);

        // Measurement covariance S_z via QR
        Eigen::Matrix<double, ZDim, NumSigma - 1> Zdev;
        for (int i = 1; i < NumSigma; ++i) Zdev.col(i - 1) = std::sqrt(w_c(i)) * (Zsigmas.col(i) - z_pred);

        Eigen::HouseholderQR<Eigen::Matrix<double, ZDim, NumSigma - 1>> qr(Zdev);
        Eigen::Matrix<double, ZDim, ZDim> S_z = qr.householderQ() * ZMat::Identity();

        // Apply cholupdate with 0-th deviation
        Eigen::Matrix<double, ZDim, 1> z0_dev = Zsigmas.col(0) - z_pred;
        S_z = cholupdate<ZDim>(S_z, std::sqrt(std::abs(w_c(0))) * z0_dev, w_c(0) > 0);

        // Add measurement noise
        ZMat S_r = Meas::getSqrtR();  // or passed in from caller
        for (int i = 0; i < ZDim; ++i) S_z = cholupdate<ZDim>(S_z, S_r.col(i), true);

        // Compute cross covariance P_xz
        Eigen::Matrix<double, StateDim, ZDim> P_xz = Eigen::Matrix<double, StateDim, ZDim>::Zero();
        for (int i = 0; i < NumSigma; ++i)
            P_xz += w_c(i) * (sigmaPoints.col(i) - state_.x) * (Zsigmas.col(i) - z_pred).transpose();

        // Kalman gain
        Eigen::Matrix<double, ZDim, ZDim> S_z_inv = S_z.inverse();
        Eigen::Matrix<double, StateDim, ZDim> K = P_xz * S_z_inv.transpose() * S_z_inv;

        // Update state
        state_.x += K * (z - z_pred);

        // Update square-root covariance
        Eigen::Matrix<double, StateDim, ZDim> KSz = K * S_z;
        for (int i = 0; i < ZDim; ++i) S_x = cholupdate<StateDim>(S_x, KSz.col(i), false);

        state_.P = S_x * S_x.transpose();  // or keep S_x if working in sqrt form
    }

    /*! Compute the measurement residuals if the measurement data was fresh.
     * The post fits are y - ybar if a measurement was read, if observations are not present
     * a flag is raised to not compute post fit residuals
    @param Measurement
    @return Eigen::VectorXd
     */
    Eigen::VectorXd computeResiduals(const std::unique_ptr<IMeasurement<StateDim>> measurement) {
        /*! - Compute Post Fit Residuals, first get Y (eq 22) using the states post fit*/
        Eigen::MatrixXd yMeas(measurement.size(), this->numberSigmaPoints);
        for (size_t j = 0; j < this->numberSigmaPoints; ++j) {
            /*! Sigma points positions need to be normalized for the measurement model.*/
            yMeas.col(j) = measurement.model(this->sigmaPoints[j]);
        }
        /*! - Compute the value for the yBar parameter (equation 23)*/
        Eigen::VectorXd yBar;
        yBar.setZero(measurement.size());
        for (size_t i = 0; i < this->numberSigmaPoints; ++i) {
            yBar += this->wM(i) * yMeas.col(i);
        }
        return measurement.getObservation() - yBar;
    }

    const StateType& getStateImpl() const { return state_; }
    void setStateImpl(const StateType& s) { state_ = s; }

    /*! Set the filter alpha parameter
    @param double alphaInput
    @return void
    */
    void setAlpha(const double alphaInput) {
        this->alpha = alphaInput;
        this->computeLambdaAndEta();
    }

    /*! Get the filter alpha parameter
        @return double alpha
        */
    double getAlpha() const { return this->alpha; }

    /*! Set the filter beta parameter
        @param double betaInput
        @return void
        */
    void setBeta(const double betaInput) { this->beta = betaInput; }

    /*! Get the filter beta parameter
        @return double beta
        */
    double getBeta() const { return this->beta; }

   private:
    StateType state_;

    double beta = 2.0;
    double alpha = 1e-3;
    double lambda{};
    double eta{};

    template <int N>
    Eigen::Matrix<double, N, N> cholupdate(Eigen::Matrix<double, N, N> L,
                                           const Eigen::Matrix<double, N, 1>& x,
                                           bool update = true) {
        Eigen::Matrix<double, N, 1> v = x;
        double sign = update ? 1.0 : -1.0;

        for (int i = 0; i < N; ++i) {
            double r = std::sqrt(L(i, i) * L(i, i) + sign * v(i) * v(i));
            double c = r / L(i, i);
            double s = v(i) / L(i, i);

            L(i, i) = r;
            for (int j = i + 1; j < N; ++j) {
                L(j, i) = (L(j, i) + sign * s * v(j)) / c;
                v(j) = c * v(j) - s * L(j, i);
            }
        }

        return L;
    }

    void computeLambdaAndEta() {
        this->lambda = StateDim * (alpha * alpha - 1);
        this->eta = std::sqrt(StateDim + lambda);
    }
};

#endif  // BASILISK_SRUNSCENTEDKALMANFILTER_H
