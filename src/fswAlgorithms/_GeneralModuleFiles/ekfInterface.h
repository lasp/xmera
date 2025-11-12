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

/*! Enumerator class to set if the filter is meant to be used purely as a linear/classical KF,
 * no reference state updates will be performed in the classical filter, while the EKF updates the reference
 * with the computed state error at each measurement update */
enum class FilterType { Classical, Extended };

template<typename StateVector>
struct EkfMeasurementModel {
public:
    EkfMeasurementModel() = default;
    virtual ~EkfMeasurementModel() = default;

    //! [-] observation measurement model
    virtual Eigen::MatrixXd model(const StateVector& state) const = 0;

    virtual Eigen::MatrixXd measurementMatrix(const StateVector& state) const = 0;

    virtual Eigen::VectorXd subtract(
        const Eigen::VectorXd& observed,
        const Eigen::VectorXd& predicted
    ) const = 0;


    virtual Eigen::VectorXd getObservation() const = 0;

    virtual Eigen::MatrixXd getNoise() const = 0;

    virtual void setPreFitResiduals(Eigen::VectorXd const& preFitResiduals) = 0;

    virtual void setPostFitResiduals(Eigen::VectorXd const& postFitResiduals) = 0;
};

namespace xmera {
    template<typename StateVector>
    concept ekf_state_vector = requires(StateVector const constState) {
        requires xmera::linearly_combinable<StateVector>;
        requires xmera::has_position<StateVector>;
        requires xmera::has_velocity<StateVector>;
    };
}

/*! @brief Extended or Classical/Linear Kalman Filter base class. */
template<xmera::ekf_state_vector StateVector>
class EkfInterface final : public KalmanFilter<EkfMeasurementModel<StateVector>> {
public:
    //! [-] State variable for logging
    StateVector stateLogged;
    //! [-] Current mean state error
    Eigen::VectorXd stateError;

    //! [-] Dynamics to compute the state derivative at a time
    DynamicsModel<StateVector> dynamics;
    std::function<Eigen::MatrixXd(double time, StateVector state)> dynamicsTransitionMatrix;
    //! [-] process noise matrix
    Eigen::MatrixXd processNoise;

    //! [-] State estimate for time TimeTag
    StateVector state;
    //! [-] covariance
    Eigen::MatrixXd covar;

    //! [-] State estimate for time TimeTag at previous time
    StateVector stateInitial;
    //! [-] covariance at previous time
    Eigen::MatrixXd covarInitial;
    //! [-] Scale that converts input units (SI) to a desired unit for the inner maths
    double unitConversion = 1;

private:
    //! [-] Infinite norm after which the filter will begin processing measurements as an extended kalman filter
    double minCovarNorm = 1E-5;

    //! [-] flag to know whether the filter is being run as a linear KF or extended KF
    FilterType filterType = FilterType::Extended;

private:
    struct StateWithStm final {
        StateVector state;
        Eigen::MatrixXd stm;

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
        this->minCovarNorm = infiniteNorm;
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
        assert(this->stateInitial.size() == this->covarInitial.rows() &&
               this->stateInitial.size() == this->covarInitial.cols());

        this->state = this->stateInitial.scale(this->unitConversion);
        this->covar = this->unitConversion * this->unitConversion * this->covarInitial;
        this->covar.resize(this->state.size(), this->state.size());

        this->stateError = Eigen::VectorXd::Zero(this->state.size());
        this->stateLogged = this->state;
        this->processNoise.resize(this->state.getPositionStates().size(), this->state.getPositionStates().size());
        this->minCovarNorm = this->minCovarNorm * this->unitConversion * this->unitConversion;
    }

    /*! Perform the time update for kalman filter.
     @return void
     @param updateTime The time since last update that we need to fix the filter to (seconds)
     */
    void timeUpdate(const double dt) override {
        /*! - Propagate full state and STM vector using dynamics specified in child class */
        Eigen::MatrixXd stateTransitionMatrix;
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
                .stm = Eigen::MatrixXd::Identity(this->state.size(), this->state.size()),
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
        Eigen::MatrixXd processNoiseMapping;
        processNoiseMapping.setZero(this->state.size(), this->state.getPositionStates().size());
        processNoiseMapping.block(0,
                                  0,
                                  this->state.getPositionStates().size(),
                                  this->state.getPositionStates().size()) =
            pow(dt, 2) / 2 *
            Eigen::MatrixXd::Identity(this->state.getPositionStates().size(), this->state.getPositionStates().size());
        processNoiseMapping.block(this->state.getPositionStates().size(),
                                  0,
                                  this->state.getVelocityStates().size(),
                                  this->state.getPositionStates().size()) =
            dt *
            Eigen::MatrixXd::Identity(this->state.getVelocityStates().size(), this->state.getVelocityStates().size());

        this->covar = stateTransitionMatrix * this->covar * stateTransitionMatrix.transpose() +
                      processNoiseMapping * this->processNoise * processNoiseMapping.transpose();
    }

    /*! Perform the measurement update for the kalman filter.
     @param Measurement
     @return void
     */
    void measurementUpdate(EkfMeasurementModel<StateVector> &measurement) override {
        auto const& observation = measurement.getObservation();

        auto const& preFitResiduals = this->computeResiduals(measurement);
        measurement.setPreFitResiduals(preFitResiduals);

        /*! - Compute the measurement matrix at this state */
        Eigen::MatrixXd measurementMatrix = measurement.measurementMatrix(this->state);

        /*! - Compute the Kalman Gain */
        Eigen::MatrixXd kalmanGain =
            this->computeKalmanGain(this->covar, measurementMatrix, measurement.getNoise());

        /*! - Update the covariance */
        this->updateCovariance(measurementMatrix, measurement.getNoise(), kalmanGain);

        if ((this->covar.maxCoeff() > this->minCovarNorm && this->filterType == FilterType::Extended) ||
            this->filterType == FilterType::Classical) {
            /*! - Compute the update with a CKF if the covariance is high at the time of the update to avoid divergence*/
            this->ckfUpdate(kalmanGain, preFitResiduals);
        } else {
            /*! - Compute the valid observations delta */
            Eigen::VectorXd measurementDelta =
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
    Eigen::VectorXd computeResiduals(const EkfMeasurementModel<StateVector> &measurement) {
        Eigen::VectorXd measurementDelta = measurement.subtract(
            measurement.getObservation(),
            measurement.model(this->state)
        );

        Eigen::MatrixXd measurementMatrix = measurement.measurementMatrix(this->state);

        return measurement.subtract(measurementDelta, measurementMatrix * this->stateError);
    }

    /*! Compute the Kalman Gain
     @param Eigen::MatrixXd covar
     @param Eigen::MatrixXd measurementMatrix
     @param Eigen::MatrixXd measurementNoise
     @return Eigen::MatrixXd
     */
    Eigen::MatrixXd computeKalmanGain(
        const Eigen::MatrixXd &covariance,
        const Eigen::MatrixXd &measurementMatrix,
        const Eigen::MatrixXd &measurementNoise
    ) const {
        Eigen::MatrixXd kalmanGain(covariance.cols(), measurementNoise.cols());
        kalmanGain = covariance * measurementMatrix.transpose();
        kalmanGain *= (measurementMatrix * covariance * measurementMatrix.transpose() + measurementNoise).inverse();
        return kalmanGain;
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
        Eigen::MatrixXd josephTransform(this->state.size(), this->state.size());
        josephTransform = Eigen::MatrixXd::Identity(this->state.size(), this->state.size()) - kalmanGain * measMat;
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
