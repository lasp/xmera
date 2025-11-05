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

public:
    // Inheritors implement these to configure the appropriate behaviors of this
    // generic Kalman filter to a specific setting.
    virtual void reset();
    virtual void timeUpdate(double dt) = 0;
    virtual Eigen::MatrixXd measurementUpdate(const MeasurementModel& measurement) = 0;
    virtual Eigen::VectorXd computeResiduals(const MeasurementModel& measurement) = 0;

public:
    // These fields represent configuration-time properties of a Kalman filter.
    // They are only used in ::reset (both in this class and derived classes).
    FilterStateVector stateInitial;  //!< [-] State estimate for time TimeTag at previous time
    Eigen::MatrixXd covarInitial;  //!< [-] covariance at previous time
    double unitConversion = 1;    //!< [-] Scale that converts input units (SI) to a desired unit for the inner maths

public:
    // These fields represent configuration-time properties of a Kalman filter.
    // They are only used by implementors of this class (and potentially clients thereof).
    DynamicsModel dynamics;
    Eigen::MatrixXd processNoise;  //!< [-] process noise matrix

public:
    // These fields are operated upon *only* by the abstract methods given
    // by any particular implementor of KalmanFilter.
    //
    // These fields represent the internal state of a generic Kalman filter.
    // They are accessed by both the implementors of this class
    // and by the module that drives the filter.
    FilterStateVector state;       //!< [-] State estimate for time TimeTag
    Eigen::MatrixXd covar;         //!< [-] covariance
};

namespace xmera {
    using MeasurementVector = std::array<std::optional<MeasurementModel>, MAX_MEASUREMENT_NUMBER>;

    void updateKalmanFilter(
        KalmanFilter& filterState,
        MeasurementVector measurements,
        uint64_t previousSimNanos,
        uint64_t currentSimNanos
    );
}

#endif /* KALMAN_FILTER_INTERFACE_HPP */
