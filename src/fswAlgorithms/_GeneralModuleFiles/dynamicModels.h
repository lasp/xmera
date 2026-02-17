// SPDX-License-Identifier: ISC
// Copyright (c) 2024, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

#ifndef FILTER_DYN_MODELS_H
#define FILTER_DYN_MODELS_H

#include <Eigen/Core>
#include <fswAlgorithms/_GeneralModuleFiles/stateModels.h>

/*! @brief Measurement models used to map a state vector to a measurement */
class DynamicsModel {
   private:
    std::function<const FilterStateVector(const double, const FilterStateVector&)>
        propagator;  //!< [-] state propagator using dynamics
    std::function<const Eigen::MatrixXd(const double, const FilterStateVector&)>
        dynamicsMatrix;  //!< [-] partial of dynamics wrt state

    static FilterStateVector rk4(
        std::function<const FilterStateVector(const double, const FilterStateVector&)> ODEfunction,
        const FilterStateVector& X0,
        double t0,
        double dt);

   public:
    DynamicsModel() = default;
    ~DynamicsModel() = default;

    FilterStateVector propagate(std::array<double, 2>, const FilterStateVector& state, double dt) const;
    void setDynamics(
        const std::function<const FilterStateVector(const double, const FilterStateVector&)>& dynamicsPropagator);

    Eigen::MatrixXd computeDynamicsMatrix(double time, const FilterStateVector& state) const;
    void setDynamicsMatrix(
        const std::function<const Eigen::MatrixXd(const double, const FilterStateVector&)>& dynamicsMatrixCalculator);
};

#endif
