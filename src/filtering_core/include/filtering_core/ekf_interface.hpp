// SPDX-License-Identifier: ISC
// Copyright (c) 2026, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

#ifndef FILTERING_CORE_EKF_INTERFACE_HPP
#define FILTERING_CORE_EKF_INTERFACE_HPP

#include <filtering_core/concepts.hpp>
#include <filtering_core/dynamics_model.hpp>

#include <Eigen/Core>
#include <Eigen/Dense>

#include <functional>

namespace filtering {

// Bundled position+velocity state for EKF use. SIZE is the dimensionality
// of each component; the full state has dimension 2*SIZE.
template<Eigen::Index SIZE>
struct EkfStateVector final {
    static_assert(SIZE != Eigen::Dynamic);

    Eigen::Vector<double, SIZE> position;
    Eigen::Vector<double, SIZE> velocity;

    static constexpr Eigen::Index size = 2 * SIZE;
    using Storage = Eigen::Vector<double, size>;

    Storage raw() const {
        Storage out;
        out.template head<SIZE>() = this->position;
        out.template tail<SIZE>() = this->velocity;
        return out;
    }

    EkfStateVector add(EkfStateVector const& other) const {
        return {
            .position = this->position + other.position,
            .velocity = this->velocity + other.velocity,
        };
    }

    EkfStateVector scale(double scalar) const {
        return {
            .position = this->position * scalar,
            .velocity = this->velocity * scalar,
        };
    }

    EkfStateVector addVector(Storage const& values) const {
        return {
            .position = this->position + values.template head<SIZE>(),
            .velocity = this->velocity + values.template tail<SIZE>(),
        };
    }
};

// Pure-linear (Classical) vs. iteratively-relinearizing (Extended) Kalman
// filter. Switches per-update based on covariance norm.
enum class FilterType { Classical, Extended };

// Extended (or Classical / Linear) Kalman filter, parametrized on state
// component size and bounded measurement vector size. Holds full mutable
// filter state — mean, covariance, dynamics, process noise, tunables —
// directly as data members.
//
// Uses the Updateable<Filter, Measurement> concept (from kalman_filter.hpp)
// implicitly via `measurement_queue::applyToFilter` calls. The Measurement
// type is supplied at the `measurementUpdate(M&)` call site; any type
// satisfying `Measurement<M, EkfStateVector<STATE_SIZE>>` works.
template<Eigen::Index STATE_SIZE, Eigen::Index MAX_MEAS_SIZE = Eigen::Dynamic>
class EkfInterface final {
    static_assert(STATE_SIZE != Eigen::Dynamic);

    using VectorMd = Eigen::Matrix<double, Eigen::Dynamic, 1,
                                   0, MAX_MEAS_SIZE, 1>;
    using MatrixMMd = Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic,
                                    0, MAX_MEAS_SIZE, MAX_MEAS_SIZE>;

    using VectorSd  = Eigen::Vector<double, 2 * STATE_SIZE>;
    using MatrixSSd = Eigen::Matrix<double, 2 * STATE_SIZE, 2 * STATE_SIZE>;

    using MatrixSMd = Eigen::Matrix<double, 2 * STATE_SIZE, Eigen::Dynamic,
                                    0, 2 * STATE_SIZE, MAX_MEAS_SIZE>;
    using MatrixMSd = Eigen::Matrix<double, Eigen::Dynamic, 2 * STATE_SIZE,
                                    0, MAX_MEAS_SIZE, 2 * STATE_SIZE>;

public:
    EkfStateVector<STATE_SIZE> stateLogged;
    VectorSd                   stateError;

    DynamicsModel<EkfStateVector<STATE_SIZE>>                                  dynamics;
    std::function<MatrixSSd(double, EkfStateVector<STATE_SIZE>)>               dynamicsTransitionMatrix;
    std::function<MatrixSSd(double)>                                           processNoise;

    EkfStateVector<STATE_SIZE> state;
    MatrixSSd                  covar;

    EkfStateVector<STATE_SIZE> stateInitial;
    MatrixSSd                  covarInitial;

    double unitConversion = 1;

private:
    double     minCovarNorm = 1E-5;
    FilterType filterType   = FilterType::Extended;

    struct StateWithStm final {
        EkfStateVector<STATE_SIZE> state;
        MatrixSSd                  stm;

        StateWithStm add(StateWithStm const& other) const {
            return { .state = this->state.add(other.state), .stm = this->stm + other.stm };
        }
        StateWithStm scale(double scalar) const {
            return { .state = this->state.scale(scalar),    .stm = this->stm * scalar };
        }
    };
    static_assert(LinearlyCombinable<StateWithStm>);

public:
    explicit EkfInterface(FilterType type) { this->filterType = type; }

    void setMinimumCovarianceNormForEkf(double infiniteNorm) {
        this->minCovarNorm = infiniteNorm * this->unitConversion * this->unitConversion;
    }
    double getMinimumCovarianceNormForEkf() const {
        return this->minCovarNorm / this->unitConversion / this->unitConversion;
    }

    void reset() {
        this->state = this->stateInitial.scale(this->unitConversion);
        this->covar = this->unitConversion * this->unitConversion * this->covarInitial;

        this->stateError.setZero();
        this->stateLogged = this->state;
        this->minCovarNorm = this->minCovarNorm * this->unitConversion * this->unitConversion;
    }

    void timeUpdate(double dt) {
        MatrixSSd stateTransitionMatrix;
        {
            DynamicsModel<StateWithStm> stateStmDynamics =
                [this](double time, StateWithStm const& s) -> StateWithStm {
                    return {
                        .state = this->dynamics(time, s.state),
                        .stm   = this->dynamicsTransitionMatrix(time, s.state) * s.stm,
                    };
                };
            StateWithStm stateStm = { .state = this->state, .stm = MatrixSSd::Identity() };
            auto propagated = filtering::propagate(stateStmDynamics, stateStm, {0, dt}, dt);

            stateTransitionMatrix = propagated.stm;
            this->state = propagated.state;
        }

        this->stateError  = stateTransitionMatrix * this->stateError;
        this->stateLogged = this->state.addVector(this->stateError);

        this->covar = stateTransitionMatrix * this->covar * stateTransitionMatrix.transpose()
                    + this->processNoise(dt);
    }

    template<class M>
        requires Measurement<M, EkfStateVector<STATE_SIZE>>
    void measurementUpdate(M& measurement) {
        VectorMd const& observation     = measurement.observation();
        VectorMd const& preFitResiduals = this->computeResiduals(measurement);
        measurement.setPreFitResiduals(preFitResiduals);

        MatrixMMd measurementMatrix = measurement.measurementMatrix(this->state);

        MatrixSMd kalmanGain = this->computeKalmanGain(this->covar, measurementMatrix, measurement.noise());

        this->updateCovariance(measurementMatrix, measurement.noise(), kalmanGain);

        bool const useClassical =
            (this->covar.maxCoeff() > this->minCovarNorm && this->filterType == FilterType::Extended)
            || this->filterType == FilterType::Classical;

        if (useClassical) {
            this->ckfUpdate(kalmanGain, preFitResiduals);
        } else {
            VectorMd measurementDelta = measurement.subtract(observation, measurement.model(this->state));
            this->ekfUpdate(kalmanGain, measurementDelta);
        }

        measurement.setPostFitResiduals(measurement.subtract(
            observation,
            measurement.model(this->state)
        ));
    }

private:
    template<class M>
    VectorMd computeResiduals(M const& measurement) {
        auto measurementDelta = measurement.subtract(
            measurement.observation(),
            measurement.model(this->state)
        );
        auto measurementMatrix = measurement.measurementMatrix(this->state);
        return measurement.subtract(measurementDelta, measurementMatrix * this->stateError);
    }

    MatrixSMd computeKalmanGain(
        MatrixSSd const& covariance,
        MatrixMSd const& measurementMatrix,
        MatrixMMd const& measurementNoise
    ) const {
        return (covariance * measurementMatrix.transpose())
             * (measurementMatrix * covariance * measurementMatrix.transpose() + measurementNoise).inverse();
    }

    void updateCovariance(
        MatrixMSd const& measMat,
        MatrixMMd const& noise,
        MatrixSMd const& kalmanGain
    ) {
        auto josephTransform = MatrixSSd::Identity() - kalmanGain * measMat;
        this->covar = josephTransform * this->covar * josephTransform.transpose();
        this->covar += kalmanGain * noise * kalmanGain.transpose();
    }

    void ckfUpdate(MatrixSMd const& kalmanGain, VectorMd const& residual) {
        this->stateError  = this->stateError + kalmanGain * residual;
        this->stateLogged = this->state.addVector(this->stateError);
    }

    void ekfUpdate(MatrixSMd const& kalmanGain, VectorMd const& measurementDelta) {
        this->stateError  = kalmanGain * measurementDelta;
        this->state       = this->state.addVector(this->stateError);
        this->stateLogged = this->state;
    }
};

}  // namespace filtering

#endif
