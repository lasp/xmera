// SPDX-License-Identifier: ISC
// Copyright (c) 2024, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

#ifndef EKF_INTERFACE_HPP
#define EKF_INTERFACE_HPP

#include <architecture/_GeneralModuleFiles/sys_model.h>
#include <architecture/messaging/messaging.h>
#include <architecture/utilities/eigenSupport.h>
#include <architecture/utilities/macroDefinitions.h>
#include <fswAlgorithms/_GeneralModuleFiles/dynamicModels.h>
#include <fswAlgorithms/_GeneralModuleFiles/filterInterfaceDefinitions.h>
#include <fswAlgorithms/_GeneralModuleFiles/kalmanFilter.h>
#include <fswAlgorithms/_GeneralModuleFiles/stateModels.h>
#include <Eigen/Dense>
#include <array>
#include <functional>
#include <optional>

template<Eigen::Index SIZE>
struct EkfStateVector final {
    static_assert(SIZE != Eigen::Dynamic);

    Eigen::Vector<double, SIZE> position;
    Eigen::Vector<double, SIZE> velocity;

    size_t size() const {
        return this->position.size();
    }

    EkfStateVector add(EkfStateVector const& other) const {
        return {
            .position = this->position + other.position,
            .velocity = this->velocity + other.velocity,
        };
    }

    EkfStateVector scale(double scale) const {
        return {
            .position = this->position * scale,
            .velocity = this->velocity * scale,
        };
    }

    EkfStateVector addVector(Eigen::Vector<double, 2 * SIZE> values) const {
        return {
            .position = this->position + values.segment(0, this->position.size()),
            .velocity = this->velocity + values.segment(this->position.size(), this->velocity.size()),
        };
    }
};

/*! Enumerator class to set if the filter is meant to be used purely as a linear/classical KF,
 * no reference state updates will be performed in the classical filter, while the EKF updates the reference
 * with the computed state error at each measurement update */
enum class FilterType { Classical, Extended };

template<Eigen::Index SIZE>
struct EkfMeasurementModel {
public:
    EkfMeasurementModel() = default;
    virtual ~EkfMeasurementModel() = default;

    //! [-] observation measurement model
    virtual Eigen::MatrixXd model(const EkfStateVector<SIZE>& state) const = 0;

    virtual Eigen::MatrixXd measurementMatrix(const EkfStateVector<SIZE>& state) const = 0;

    virtual Eigen::VectorXd subtract(
        const Eigen::VectorXd& observed,
        const Eigen::VectorXd& predicted
    ) const = 0;


    virtual Eigen::VectorXd getObservation() const = 0;

    virtual Eigen::MatrixXd getNoise() const = 0;

    virtual void setPreFitResiduals(Eigen::VectorXd const& preFitResiduals) = 0;

    virtual void setPostFitResiduals(Eigen::VectorXd const& postFitResiduals) = 0;
};

/*! @brief Extended or Classical/Linear Kalman Filter base class. */
template<Eigen::Index SIZE>
class EkfInterface final : public KalmanFilter<EkfMeasurementModel<SIZE>> {
    static_assert(SIZE != Eigen::Dynamic);
public:
    //! [-] State variable for logging
    EkfStateVector<SIZE> stateLogged;
    //! [-] Current mean state error
    Eigen::Vector<double, 2 * SIZE> stateError;

    //! [-] Dynamics to compute the state derivative at a time
    DynamicsModel<EkfStateVector<SIZE>> dynamics;
    std::function<Eigen::MatrixXd(double time, EkfStateVector<SIZE> state)> dynamicsTransitionMatrix;
    //! [-] process noise matrix
    Eigen::Matrix<double, SIZE, SIZE> processNoise;

    //! [-] State estimate for time TimeTag
    EkfStateVector<SIZE> state;
    //! [-] covariance
    Eigen::Matrix<double, 2 * SIZE, 2 * SIZE> covar;

    //! [-] State estimate for time TimeTag at previous time
    EkfStateVector<SIZE> stateInitial;
    //! [-] covariance at previous time
    Eigen::Matrix<double, 2 * SIZE, 2 * SIZE> covarInitial;
    //! [-] Scale that converts input units (SI) to a desired unit for the inner maths
    double unitConversion = 1;

private:
    //! [-] Infinite norm after which the filter will begin processing measurements as an extended kalman filter
    double minCovarNorm = 1E-5;

    //! [-] flag to know whether the filter is being run as a linear KF or extended KF
    FilterType filterType = FilterType::Extended;

private:
    struct StateWithStm final {
        EkfStateVector<SIZE> state;
        Eigen::Matrix<double, 2 * SIZE, 2 * SIZE> stm;

        StateWithStm add(StateWithStm const& other) const {
            return {
                .state = this->state.add(other.state),
                .stm = this->stm + other.stm,
            };
        }

        StateWithStm scale(double scale) const {
            return {
                .state = this->state.scale(scale),
                .stm = this->stm * scale,
            };
        }
    };

    static_assert(xmera::linearly_combinable<StateWithStm>);

public:
    explicit EkfInterface(const FilterType type)
        : filterType(type)
    {}

    ~EkfInterface() override = default;

    /*! Set a minimum value (infinite norm, meaning maximal term) of the covariance before switching to Extended KF updates.
     *  This prevents divergence if the initial covariance is high and the state changes too abruptly
     @param double infiniteNorm
     @return void
     */
    void setMinimumCovarianceNormForEkf(const double infiniteNorm) {
        this->minCovarNorm = infiniteNorm * this->unitConversion * this->unitConversion;
    }

    /*! Get the minimum value of the covariance before switching to Extended KF updates.
     @return double infiniteNorm
     */
    double getMinimumCovarianceNormForEkf() const {
        return this->minCovarNorm / this->unitConversion / this->unitConversion;
    }

public:
    /*! Reset all of the filter states, including the custom reset
     @return void
     */
    void reset() override {
        this->state = this->stateInitial.scale(this->unitConversion);
        this->covar = this->unitConversion * this->unitConversion * this->covarInitial;

        this->stateError.setZero();
        this->stateLogged = this->state;
        this->minCovarNorm = this->minCovarNorm * this->unitConversion * this->unitConversion;
    }

    /*! Perform the time update for kalman filter.
     @return void
     @param updateTime The time since last update that we need to fix the filter to (seconds)
     */
    void timeUpdate(const double dt) override {
        /*! - Propagate full state and STM vector using dynamics specified in child class */
        Eigen::Matrix<double, 2 * SIZE, 2 * SIZE> stateTransitionMatrix;
        {
            DynamicsModel<StateWithStm> stateStmDynamics =
                [this](double time, StateWithStm const& state) -> StateWithStm {
                    return {
                        .state = this->dynamics(time, state.state),
                        .stm = this->dynamicsTransitionMatrix(time, state.state) * state.stm,
                    };
                };
            StateWithStm stateStm = {
                .state = this->state,
                .stm = Eigen::Matrix<double, 2 * SIZE, 2 * SIZE>::Identity(),
            };

            auto propagatedStateStm = xmera::propagate(stateStmDynamics, stateStm, {0, dt}, dt);

            stateTransitionMatrix = propagatedStateStm.stm;
            this->state = propagatedStateStm.state;
        }

        /*! - Unpack propagated states and update state, and state error */
        this->stateError = stateTransitionMatrix * this->stateError;
        this->stateLogged = this->state.addVector(this->stateError);

        /*! - Update the covariance Pbar = Phi*P*Phi^T + Gamma*Q*Gamma^T
         * The process noise mapping will depend on the number of "rate" states */
        Eigen::Matrix<double, 2 * SIZE, SIZE> processNoiseMapping;
        processNoiseMapping.block(0, 0, SIZE, SIZE) =
            Eigen::Matrix<double, SIZE, SIZE>::Identity() * (pow(dt, 2) / 2);
        processNoiseMapping.block(SIZE, 0, SIZE, SIZE) =
            Eigen::Matrix<double, SIZE, SIZE>::Identity() * dt;

        this->covar = stateTransitionMatrix * this->covar * stateTransitionMatrix.transpose() +
                      processNoiseMapping * this->processNoise * processNoiseMapping.transpose();
    }

    /*! Perform the measurement update for the kalman filter.
     @param Measurement
     @return void
     */
    void measurementUpdate(EkfMeasurementModel<SIZE> &measurement) override {
        auto const& observation = measurement.getObservation();

        auto const& preFitResiduals = this->computeResiduals(measurement);
        measurement.setPreFitResiduals(preFitResiduals);

        /*! - Compute the measurement matrix at this state */
        auto measurementMatrix = measurement.measurementMatrix(this->state);

        /*! - Compute the Kalman Gain */
        auto kalmanGain = this->computeKalmanGain(this->covar, measurementMatrix, measurement.getNoise());

        /*! - Update the covariance */
        this->updateCovariance(measurementMatrix, measurement.getNoise(), kalmanGain);

        if ((this->covar.maxCoeff() > this->minCovarNorm && this->filterType == FilterType::Extended) ||
            this->filterType == FilterType::Classical) {
            /*! - Compute the update with a CKF if the covariance is high at the time of the update to avoid divergence*/
            this->ckfUpdate(kalmanGain, preFitResiduals);
        } else {
            /*! - Compute the valid observations delta */
            auto measurementDelta =
                measurement.subtract(observation, measurement.model(this->state));

            /*! - Compute the update with a EKF, the reference state is changed by the filter update */
            this->ekfUpdate(kalmanGain, measurementDelta);
        }

        measurement.setPostFitResiduals(measurement.subtract(
            observation,
            measurement.model(this->state)
        ));
    }

private:
    /*! Compute the measurement residuals at a given time.
     @param Measurement
     @return Eigen::VectorXd
     */
    Eigen::VectorXd computeResiduals(const EkfMeasurementModel<SIZE> &measurement) {
        auto measurementDelta = measurement.subtract(
            measurement.getObservation(),
            measurement.model(this->state)
        );

        auto measurementMatrix = measurement.measurementMatrix(this->state);

        return measurement.subtract(measurementDelta, measurementMatrix * this->stateError);
    }

    /*! Compute the Kalman Gain
     @param Eigen::MatrixXd covar
     @param Eigen::MatrixXd measurementMatrix
     @param Eigen::MatrixXd measurementNoise
     @return Eigen::MatrixXd
     */
    Eigen::MatrixXd computeKalmanGain(
        const Eigen::Matrix<double, 2 * SIZE, 2 * SIZE> &covariance,
        const Eigen::MatrixXd &measurementMatrix,
        const Eigen::MatrixXd &measurementNoise
    ) const {
        return
              (covariance * measurementMatrix.transpose())
            * (measurementMatrix * covariance * measurementMatrix.transpose() + measurementNoise).inverse();
    }

    /*! Update the covariance using the Joseph form of the update
     @param Eigen::MatrixXd measMat
     @param Eigen::MatrixXd noise
     @param Eigen::MatrixXd kalmanGain
     @return void
     */
    void updateCovariance(
        const Eigen::MatrixXd &measMat,
        const Eigen::MatrixXd &noise,
        const Eigen::MatrixXd &kalmanGain
    ) {
        auto josephTransform = Eigen::MatrixXd::Identity(this->state.size(), this->state.size()) - kalmanGain * measMat;
        this->covar = josephTransform * this->covar * josephTransform.transpose();
        this->covar += kalmanGain * noise * kalmanGain.transpose();
    }

    /*! Classical Kalman Filter Update (the reference state is unchanged)
     @param Eigen::MatrixXd kalmanGain
     @param Eigen::VectorXd residual
     @return void
     */
    void ckfUpdate(const Eigen::MatrixXd &kalmanGain, const Eigen::VectorXd &residual) {
        this->stateError = this->stateError + kalmanGain * residual;
        this->stateLogged = this->state.addVector(this->stateError);
    }

    /*! Extended Kalman Filter Update (the reference state is updated given the state error)
     @param Eigen::MatrixXd kalmanGain
     @param Eigen::VectorXd measurementDelta
     @return void
     */
    void ekfUpdate(const Eigen::MatrixXd &kalmanGain, const Eigen::VectorXd &measurementDelta) {
        this->stateError = kalmanGain * measurementDelta;

        this->state = this->state.addVector(this->stateError);
        this->stateLogged = this->state;
    }
};

#endif /* EKF_INTERFACE_HPP */
