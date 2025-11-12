// SPDX-License-Identifier: ISC
// Copyright (c) 2024, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

#include "ekfInterface.h"

struct StateWithStm final {
    FilterStateVector state;
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


EkfInterface::EkfInterface(const FilterType type) { this->filterType = type; }

/*! Reset all of the filter states, including the custom reset
 @return void
 */
void EkfInterface::reset() {
    assert(this->stateInitial.size() == this->covarInitial.rows() &&
           this->stateInitial.size() == this->covarInitial.cols());

    this->state = this->stateInitial.scale(this->unitConversion);
    this->covar = this->unitConversion * this->unitConversion * this->covarInitial;
    this->covar.resize(this->state.size(), this->state.size());

    this->stateError = Eigen::VectorXd::Zero(this->state.size());
    this->stateLogged = this->state;
    this->stateTransitionMatrix = Eigen::MatrixXd::Identity(this->state.size(), this->state.size());
    if (this->stateInitial.hasVelocity()) {
        this->processNoise.resize(this->state.getVelocityStates().size(), this->state.getVelocityStates().size());
    } else {
        this->processNoise.resize(this->state.getPositionStates().size(), this->state.getPositionStates().size());
    }
    this->minCovarNorm = this->minCovarNorm * this->unitConversion * this->unitConversion;
}

/*! Perform the time update for kalman filter.
 @return void
 @param updateTime The time since last update that we need to fix the filter to (seconds)
 */
void EkfInterface::timeUpdate(const double dt) {
    this->stateTransitionMatrix = Eigen::MatrixXd::Identity(this->state.size(), this->state.size());
    std::array<double, 2> time = {0, dt};

    /*! - Propagate full state and STM vector using dynamics specified in child class */
    DynamicsModel<StateWithStm> stateStmDynamics = [this](double time, StateWithStm const& state) -> StateWithStm {
        return {
            .state = this->dynamics(time, state.state),
            .stm = this->dynamicsTransitionMatrix(time, state.state) * state.stm,
        };
    };
    StateWithStm stateStm = {
        .state = this->state,
        .stm = this->stateTransitionMatrix,
    };
    StateWithStm propagatedStateStm = xmera::propagate(stateStmDynamics, stateStm, time, dt);

    /*! - Unpack propagated states and update state, and state error */
    this->stateTransitionMatrix = propagatedStateStm.stm;
    this->state = propagatedStateStm.state;
    this->stateError = this->stateTransitionMatrix * this->stateError;
    this->stateLogged = this->state.addVector(this->stateError);

    /*! - Update the covariance Pbar = Phi*P*Phi^T + Gamma*Q*Gamma^T
     * The process noise mapping will depend on the number of "rate" states */
    Eigen::MatrixXd processNoiseMapping;
    if (!this->state.hasVelocity()) {
        processNoiseMapping =
            Eigen::MatrixXd::Identity(this->state.getPositionStates().size(), this->state.getPositionStates().size());
        processNoiseMapping *= pow(dt, 2) / 2;
    } else {
        processNoiseMapping.setZero(this->state.getPositionStates().size() + this->state.getVelocityStates().size(),
                                    this->state.getVelocityStates().size());
        processNoiseMapping.block(
            0, 0, this->state.getPositionStates().size(), this->state.getPositionStates().size()) =
            pow(dt, 2) / 2 *
            Eigen::MatrixXd::Identity(this->state.getPositionStates().size(), this->state.getPositionStates().size());
        processNoiseMapping.block(this->state.getPositionStates().size(),
                                  0,
                                  this->state.getVelocityStates().size(),
                                  this->state.getVelocityStates().size()) =
            dt *
            Eigen::MatrixXd::Identity(this->state.getVelocityStates().size(), this->state.getVelocityStates().size());
    }
    this->covar = this->stateTransitionMatrix * this->covar * this->stateTransitionMatrix.transpose() +
                  processNoiseMapping * this->processNoise * processNoiseMapping.transpose();
}

/*! Perform the measurement update for the kalman filter.
 @param Measurement
 @return void
 */
void EkfInterface::measurementUpdate(EkfMeasurementModel& measurement) {
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

/*! Compute the Kalman Gain
@param Eigen::MatrixXd covar
@param Eigen::MatrixXd measurementMatrix
@param Eigen::MatrixXd measurementNoise
@return Eigen::MatrixXd
 */
Eigen::MatrixXd EkfInterface::computeKalmanGain(const Eigen::MatrixXd& covariance,
                                                const Eigen::MatrixXd& measurementMatrix,
                                                const Eigen::MatrixXd& measurementNoise) const {
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
void EkfInterface::updateCovariance(const Eigen::MatrixXd& measMat,
                                    const Eigen::MatrixXd& noise,
                                    const Eigen::MatrixXd& kalmanGain) {
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
void EkfInterface::ckfUpdate(const Eigen::MatrixXd& kalmanGain, const Eigen::VectorXd& residual) {
    this->stateError = this->stateError + kalmanGain * residual;
    this->stateLogged = this->state.addVector(this->stateError);
}

/*! Extended Kalman Filter Update (the reference state is updated given the state error)
@param Eigen::MatrixXd kalmanGain
@param Eigen::VectorXd measurementDelta
@return void
 */
void EkfInterface::ekfUpdate(const Eigen::MatrixXd& kalmanGain, const Eigen::VectorXd& measurementDelta) {
    this->stateError = kalmanGain * measurementDelta;

    this->state = this->state.addVector(this->stateError);
    this->stateLogged = this->state;
}

/*! Compute the measurement residuals at a given time.
@param Measurement
@return Eigen::VectorXd
 */
Eigen::VectorXd EkfInterface::computeResiduals(const EkfMeasurementModel& measurement) {
    Eigen::VectorXd measurementDelta = measurement.subtract(
        measurement.getObservation(),
        measurement.model(this->state)
    );

    Eigen::MatrixXd measurementMatrix = measurement.measurementMatrix(this->state);

    return measurement.subtract(measurementDelta, measurementMatrix * this->stateError);
}

/*! Set a minimum value (infinite norm, meaning maximal term) of the covariance before switching to Extended KF updates.
 * This prevents divergence if the initial covariance is high and the state changes too abruptly
    @param double infiniteNorm
    @return void
    */
void EkfInterface::setMinimumCovarianceNormForEkf(const double infiniteNorm) { this->minCovarNorm = infiniteNorm; }

/*! Get the minimum value of the covariance before switching to Extended KF updates.
    @return double infiniteNorm
    */
double EkfInterface::getMinimumCovarianceNormForEkf() const {
    return this->minCovarNorm / this->unitConversion / this->unitConversion;
}
