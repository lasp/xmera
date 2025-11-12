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

struct EkfMeasurementModel {
public:
    EkfMeasurementModel() = default;
    virtual ~EkfMeasurementModel() = default;

    //! [-] observation measurement model
    virtual Eigen::MatrixXd model(const FilterStateVector& state) const = 0;

    virtual Eigen::MatrixXd measurementMatrix(const FilterStateVector& state) const = 0;

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
class EkfInterface final : public KalmanFilter<EkfMeasurementModel> {
public:
    explicit EkfInterface(FilterType type);
    ~EkfInterface() override = default;

    void setMinimumCovarianceNormForEkf(double infiniteNorm);
    double getMinimumCovarianceNormForEkf() const;

public:
    void reset() override;
    void timeUpdate(double dt) override;
    void measurementUpdate(EkfMeasurementModel& measurement) override;

private:
    Eigen::VectorXd computeResiduals(const EkfMeasurementModel& measurement);

    Eigen::MatrixXd computeKalmanGain(const Eigen::MatrixXd& covar,
                                      const Eigen::MatrixXd& measurementMatrix,
                                      const Eigen::MatrixXd& measurementNoise) const;

    void updateCovariance(const Eigen::MatrixXd& measMat,
                          const Eigen::MatrixXd& noise,
                          const Eigen::MatrixXd& kalmanGain);

    void ckfUpdate(const Eigen::MatrixXd& kalmanGain, const Eigen::VectorXd& residual);

    void ekfUpdate(const Eigen::MatrixXd& kalmanGain, const Eigen::VectorXd& yMeas);

public:
    //! [-] State variable for logging
    FilterStateVector stateLogged;
    //! [-] Current mean state error
    Eigen::VectorXd stateError;

    //! [-] Dynamics to compute the state derivative at a time
    DynamicsModel<FilterStateVector> dynamics;
    //! [-] process noise matrix
    Eigen::MatrixXd processNoise;

    //! [-] State estimate for time TimeTag
    FilterStateVector state;
    //! [-] covariance
    Eigen::MatrixXd covar;

    //! [-] State estimate for time TimeTag at previous time
    FilterStateVector stateInitial;
    //! [-] covariance at previous time
    Eigen::MatrixXd covarInitial;
    //! [-] Scale that converts input units (SI) to a desired unit for the inner maths
    double unitConversion = 1;

private:
    //! [-] State Transition Matrix
    Eigen::MatrixXd stateTransitionMatrix;

    //! [-] Infinite norm after which the filter will begin processing measurements as an extended kalman filter
    double minCovarNorm = 1E-5;

    //! [-] flag to know whether the filter is being run as a linear KF or extended KF
    FilterType filterType = FilterType::Extended;
};

#endif /* EKF_INTERFACE_HPP */
