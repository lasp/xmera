// SPDX-License-Identifier: ISC
// Copyright (c) 2024, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

#ifndef KALMAN_FILTER_INTERFACE_HPP
#define KALMAN_FILTER_INTERFACE_HPP

#include <fswAlgorithms/_GeneralModuleFiles/filterInterfaceDefinitions.h>
#include <fswAlgorithms/_GeneralModuleFiles/measurementModels.h>

#include <Eigen/Dense>

#include <array>
#include <optional>

/*! @brief Kalman Filter interface */
class KalmanFilter {
public:
    KalmanFilter() = default;
    virtual ~KalmanFilter() = default;

    virtual void reset() = 0;
    virtual void timeUpdate(double dt) = 0;
    virtual Eigen::MatrixXd measurementUpdate(const MeasurementModel& measurement) = 0;
    virtual Eigen::VectorXd computeResiduals(const MeasurementModel& measurement) = 0;
};

namespace xmera {
    using MeasurementVector = std::array<std::optional<MeasurementModel>, MAX_MEASUREMENT_NUMBER>;

    void updateKalmanFilter(
        KalmanFilter& filterState,
        MeasurementVector& measurements,
        uint64_t previousSimNanos,
        uint64_t currentSimNanos
    );
}

#endif /* KALMAN_FILTER_INTERFACE_HPP */
