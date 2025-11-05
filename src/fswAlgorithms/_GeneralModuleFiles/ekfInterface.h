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
#include <fswAlgorithms/_GeneralModuleFiles/measurementModels.h>
#include <fswAlgorithms/_GeneralModuleFiles/stateModels.h>
#include <Eigen/Dense>
#include <array>
#include <functional>
#include <optional>

/*! Enumerator class to set if the filter is meant to be used purely as a linear/classical KF,
 * no reference state updates will be performed in the classical filter, while the EKF updates the reference
 * with the computed state error at each measurement update */
enum class FilterType { Classical, Extended };

/*! @brief Extended or Classical/Linear Kalman Filter base class. */
class EkfInterface : public KalmanFilter {
   public:
    explicit EkfInterface(FilterType type);
    ~EkfInterface() override = default;
    void reset(uint64_t currentSimNanos) override;

    void setFilterDynamicsMatrix(
        const std::function<const Eigen::MatrixXd(const double, const FilterStateVector&)>& dynamicsMatrixCalculator);
    void setMinimumCovarianceNormForEkf(double infiniteNorm);
    double getMinimumCovarianceNormForEkf() const;

    FilterStateVector stateLogged;   //!< [-] State variable for logging
    Eigen::VectorXd stateError;      //!< [-] Current mean state error

   private:
    void timeUpdate(double dt) override;
    Eigen::MatrixXd measurementUpdate(const MeasurementModel& measurement) override;

    Eigen::VectorXd computeResiduals(const MeasurementModel& measurement) override;

    Eigen::MatrixXd computeKalmanGain(const Eigen::MatrixXd& covar,
                                      const Eigen::MatrixXd& measurementMatrix,
                                      const Eigen::MatrixXd& measurementNoise) const;
    void updateCovariance(const Eigen::MatrixXd& measMat,
                          const Eigen::MatrixXd& noise,
                          const Eigen::MatrixXd& kalmanGain);
    void ckfUpdate(const Eigen::MatrixXd& kalmanGain, const Eigen::VectorXd& residual);
    void ekfUpdate(const Eigen::MatrixXd& kalmanGain, const Eigen::VectorXd& yMeas);

    Eigen::MatrixXd stateTransitionMatrix;  //!< [-] State Transition Matrix
    double minCovarNorm = 1E-5; /*!< [-] Infinite norm after which the filter will begin processing measurements as
    an extended kalman filter */
    FilterType filterType =
        FilterType::Extended;  //!< [-] flag to know whether the filter is being run as a linear KF or extended KF
};

#endif /* EKF_INTERFACE_HPP */
