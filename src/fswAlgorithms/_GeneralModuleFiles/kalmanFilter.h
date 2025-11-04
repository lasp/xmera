// SPDX-License-Identifier: ISC
// Copyright (c) 2024, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

#ifndef KALMAN_FILTER_INTERFACE_HPP
#define KALMAN_FILTER_INTERFACE_HPP

#include <architecture/_GeneralModuleFiles/sys_model.h>
#include <architecture/messaging/messaging.h>
#include <architecture/utilities/eigenSupport.h>
#include <architecture/utilities/macroDefinitions.h>
#include <fswAlgorithms/_GeneralModuleFiles/dynamicModels.h>
#include <fswAlgorithms/_GeneralModuleFiles/filterInterfaceDefinitions.h>
#include <fswAlgorithms/_GeneralModuleFiles/measurementModels.h>
#include <fswAlgorithms/_GeneralModuleFiles/stateModels.h>
#include <Eigen/Dense>
#include <array>
#include <functional>
#include <optional>

/*! @brief Kalman Filter interface */
class KalmanFilter{
   public:
    KalmanFilter() = default;
    virtual ~KalmanFilter() = default;
    virtual void reset(uint64_t currentSimNanos);
    virtual void updateState(uint64_t currentSimNanos,
                                   std::array<std::optional<MeasurementModel>, MAX_MEASUREMENT_NUMBER> measurements);

    void setInitialPosition(const Eigen::VectorXd& initialPositionInput);
    std::optional<Eigen::VectorXd> getInitialPosition() const;
    void setInitialVelocity(const Eigen::VectorXd& initialVelocityInput);
    std::optional<Eigen::VectorXd> getInitialVelocity() const;
    void setInitialAcceleration(const Eigen::VectorXd& initialAccelerationInput);
    std::optional<Eigen::VectorXd> getInitialAcceleration() const;
    void setInitialBias(const Eigen::VectorXd& initialBiasInput);
    std::optional<Eigen::VectorXd> getInitialBias() const;
    void setInitialConsiderParameters(const Eigen::VectorXd& initialConsiderInput);
    std::optional<Eigen::VectorXd> getInitialConsiderParameters() const;

    void setInitialCovariance(const Eigen::MatrixXd& initialCovariance);
    Eigen::MatrixXd getInitialCovariance() const;
    void setProcessNoise(const Eigen::MatrixXd& processNoise);
    Eigen::MatrixXd getProcessNoise() const;
    void setUnitConversionFromSItoState(double conversion);
    double getUnitConversionFromSItoState() const;
    void setMeasurementNoiseScale(double measurementNoiseScale);
    double getMeasurementNoiseScale() const;
    void setFilterDynamics(
        const std::function<const FilterStateVector(double, const FilterStateVector&)>& dynamicsPropagator);

    FilterStateVector state;         //!< [-] State estimate for time TimeTag
    Eigen::MatrixXd covar;         //!< [-] covariance
    FilterStateVector xBar;          //!< [-] Current mean state estimate
    double previousFilterTimeTag = 0;  //!< [s]  Time tag for state covar/etc
    double measNoiseScaling = 1;  //!< [-] Scale factor for the measurement noise
    DynamicsModel dynamics;
    FilterStateVector stateLogged;   //!< [-] State variable for logging
    double unitConversion = 1;    //!< [-] Scale that converts input units (SI) to a desired unit for the inner maths
    Eigen::VectorXd stateError;      //!< [-] Current mean state error

   protected:
    virtual void timeUpdate(double updateTime) = 0;
    virtual void measurementUpdate(const MeasurementModel& measurement) = 0;
    virtual Eigen::VectorXd computeResiduals(const MeasurementModel& measurement) = 0;
    void orderMeasurementsChronologically();

    std::array<std::optional<MeasurementModel>, MAX_MEASUREMENT_NUMBER> measurements;  //!< [Measurements] All
    //!< measurement containers in chronological order
    FilterStateVector stateInitial;  //!< [-] State estimate for time TimeTag at previous time
    Eigen::MatrixXd processNoise;  //!< [-] process noise matrix
    Eigen::MatrixXd covarInitial;  //!< [-] covariance at previous time
};

#endif /* KALMAN_FILTER_INTERFACE_HPP */
