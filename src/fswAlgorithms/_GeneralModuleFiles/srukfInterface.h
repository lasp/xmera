// SPDX-License-Identifier: ISC
// Copyright (c) 2024, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

#ifndef SRUKF_INTERFACE_HPP
#define SRUKF_INTERFACE_HPP

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

struct SRukfMeasurementModel {
public:
    SRukfMeasurementModel() = default;
    virtual ~SRukfMeasurementModel() = default;

    //! [-] observation measurement model
    virtual Eigen::MatrixXd model(const FilterStateVector& state) const = 0;

    virtual Eigen::VectorXd subtract(
        const Eigen::VectorXd& observed,
        const Eigen::VectorXd& predicted
    ) const = 0;

    virtual Eigen::VectorXd getObservation() const = 0;

    virtual Eigen::MatrixXd getNoise() const = 0;

    virtual void setPreFitResiduals(Eigen::VectorXd const& preFitResiduals) = 0;

    virtual void setPostFitResiduals(Eigen::VectorXd const& postFitResiduals) = 0;
};

/*! @brief Square Root unscented Kalman Filter base class */
class SRukfInterface : public KalmanFilter<SRukfMeasurementModel> {
   public:
    SRukfInterface() = default;
    ~SRukfInterface() override = default;

    void reset() override;
    void timeUpdate(double dt) override;
    void measurementUpdate(SRukfMeasurementModel& measurement) override;

    void setAlpha(double alpha);
    double getAlpha() const;
    void setBeta(double beta);
    double getBeta() const;

    DynamicsModel dynamics;
    Eigen::MatrixXd processNoise;  //!< [-] process noise matrix
    FilterStateVector xBar{};      //!< [-] Current mean state estimate

    FilterStateVector state;       //!< [-] State estimate for time TimeTag
    Eigen::MatrixXd covar;         //!< [-] covariance

    FilterStateVector stateInitial;  //!< [-] State estimate for time TimeTag at previous time
    Eigen::MatrixXd covarInitial;    //!< [-] covariance at previous time
    double unitConversion = 1;       //!< [-] Scale that converts input units (SI) to a desired unit for the inner maths

   private:
    Eigen::VectorXd computeResiduals(const SRukfMeasurementModel& measurement);
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
