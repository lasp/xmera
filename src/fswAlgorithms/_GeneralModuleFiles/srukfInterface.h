// SPDX-License-Identifier: ISC
// Copyright (c) 2024, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

#ifndef SRUKF_INTERFACE_HPP
#define SRUKF_INTERFACE_HPP

#include <architecture/_GeneralModuleFiles/sys_model.h>
#include <architecture/messaging/messaging.h>
#include <architecture/utilities/eigenSupport.h>
#include <architecture/utilities/macroDefinitions.h>
#include <fswAlgorithms/_GeneralModuleFiles/filterInterfaceDefinitions.h>
#include <fswAlgorithms/_GeneralModuleFiles/kalmanFilter.h>
#include <fswAlgorithms/_GeneralModuleFiles/measurementModels.h>
#include <fswAlgorithms/_GeneralModuleFiles/stateModels.h>
#include <Eigen/Dense>
#include <array>
#include <functional>
#include <optional>

/*! @brief Square Root unscented Kalman Filter base class */
class SRukfInterface : public KalmanFilter {
   public:
    SRukfInterface() = default;
    ~SRukfInterface() override = default;

    void reset(uint64_t callTime) override;

    void setAlpha(double alpha);
    double getAlpha() const;
    void setBeta(double beta);
    double getBeta() const;

   private:
    void timeUpdate(double updateTime) override;
    void measurementUpdate(const MeasurementModel& measurement) override;
    Eigen::VectorXd computeResiduals(const MeasurementModel& measurement) override;

    Eigen::MatrixXd qrDecompositionJustR(const Eigen::MatrixXd& input) const;
    Eigen::MatrixXd choleskyUpDownDate(const Eigen::MatrixXd& input,
                                       const Eigen::VectorXd& inputVector,
                                       double coefficient) const;
    Eigen::MatrixXd backSubstitution(const Eigen::MatrixXd& U, const Eigen::MatrixXd& b) const;
    Eigen::MatrixXd forwardSubstitution(const Eigen::MatrixXd& L, const Eigen::MatrixXd& b) const;
    Eigen::MatrixXd choleskyDecomposition(const Eigen::MatrixXd& input) const;

    Eigen::MatrixXd sBar;                                                  //!< [-] Time updated covariance
    std::array<FilterStateVector, 2 * MAX_STATES_VECTOR + 1> sigmaPoints;  //!< [-]    sigma point vector
    size_t numberSigmaPoints = 0;                                          //!< [-] number of sigma points
    Eigen::MatrixXd cholProcessNoise;                                      //!< [-] cholesky of Qnoise
    Eigen::MatrixXd cholMeasNoise;                                         //!< [-] cholesky of Measurement noise

    double beta = 0;
    double alpha = 0;
    double lambda = 0;
    double eta = 0;

    Eigen::VectorXd wM;
    Eigen::VectorXd wC;
    Eigen::MatrixXd sBarInitial;  //!< [-] Time updated covariance at previous time
};

#endif /* SRUKF_INTERFACE_HPP */
