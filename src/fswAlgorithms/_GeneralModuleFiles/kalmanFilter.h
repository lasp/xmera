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
class KalmanFilter {
public:
    KalmanFilter() = default;
    virtual ~KalmanFilter() = default;
    virtual void reset(uint64_t currentSimNanos);

    // Used by a particular module to provide more time-ordered measurements.
    void updateState(
      uint64_t currentSimNanos,
      std::array<std::optional<MeasurementModel>, MAX_MEASUREMENT_NUMBER> measurements
    );

protected:
    // Inheritors implement these to configure the appropriate behaviors of this
    // generic Kalman filter to a specific setting.
    virtual void timeUpdate(double dt) = 0;
    virtual void measurementUpdate(const MeasurementModel& measurement) = 0;
    virtual Eigen::VectorXd computeResiduals(const MeasurementModel& measurement) = 0;

public:
    // Initialization-time methods
    // Invoked to configure a generic kalman filter for use in a particular setting
    // (e.g., by a particular xmera module)
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

    void setFilterDynamics(
        const std::function<const FilterStateVector(double, const FilterStateVector&)>& dynamicsPropagator);

public:
    // State operated upon directly by KalmanFilter::updateState
    // (and therefore meaningfully "owned" by KalmanFilter)
    FilterStateVector state;         //!< [-] State estimate for time TimeTag
    double previousFilterTimeTag = 0;  //!< [s]  Time tag for state covar/etc

public:
    // These fields represent configuration-time properties of a Kalman filter.
    // They are only used in ::reset (both in this class and derived classes).
    FilterStateVector stateInitial;  //!< [-] State estimate for time TimeTag at previous time
    double unitConversion = 1;    //!< [-] Scale that converts input units (SI) to a desired unit for the inner maths

public:
    // These fields represent configuration-time properties of a Kalman filter.
    // They are only used by implementors of this class (and potentially clients thereof).
    DynamicsModel dynamics;
    Eigen::MatrixXd processNoise;  //!< [-] process noise matrix
    Eigen::MatrixXd covarInitial;  //!< [-] covariance at previous time

public:
    // These fields are operated upon *only* by the abstract methods given
    // by any particular implementor of KalmanFilter.
    //
    // These fields represent the internal state of a generic Kalman filter.
    // They are accessed by both the implementors of this class
    // and by the module that drives the filter.
    Eigen::MatrixXd covar;         //!< [-] covariance
};

#endif /* KALMAN_FILTER_INTERFACE_HPP */
