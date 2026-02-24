// SPDX-License-Identifier: ISC

#ifndef BASILISK_STATICKALMANBASE_HPP
#define BASILISK_STATICKALMANBASE_HPP

#include "StaticFilterTraits.hpp"
#include "StaticProcessModel.hpp"

#include <type_traits>
#include <utility>

/**
 * @brief Common storage and helper utilities shared by all static-dimension Kalman filters.
 *
 * The base class owns the state estimate, covariance and process model. It provides a
 * covariance propagation helper that leverages an analytic Jacobian when the process
 * model exposes one, and falls back to a simple additive noise model otherwise.
 */
template <typename TraitsT, typename ProcessModelT>
class StaticKalmanBase {
   public:
    using Traits = TraitsT;
    using ProcessModel = ProcessModelT;

    using Scalar = typename Traits::Scalar;
    using StateVector = typename Traits::StateVector;
    using StateMatrix = typename Traits::StateMatrix;

    StaticKalmanBase() = default;
    explicit StaticKalmanBase(ProcessModel model) : processModel_(std::move(model)) {}

    const StateVector& state() const { return state_; }
    const StateMatrix& covariance() const { return covariance_; }

    void setState(const StateVector& x) { state_ = x; }
    void setCovariance(const StateMatrix& P) { covariance_ = P; }

    ProcessModel& processModel() { return processModel_; }
    const ProcessModel& processModel() const { return processModel_; }

   protected:
    /**
     * @brief Propagate the state and covariance forward in time.
     *
     * Derived filters can call this helper from their predict logic. The method updates the
     * covariance using the process model Jacobian when the model provides one; otherwise it
     * performs an additive noise update. More sophisticated propagation (e.g., sigma-point
     * based) can be implemented directly in derived filters.
     */
    void predictStateAndCovariance(double dt, const StateMatrix& processNoise) {
        const StateVector priorState = state_;
        state_ = processModel_.propagate(dt, priorState);
        propagateCovariance(dt, priorState, processNoise);
    }

    void resetToNominal() {
        state_ = Traits::zeroState();
        covariance_ = Traits::identityCovariance();
    }

    StateVector state_ = Traits::zeroState();
    StateMatrix covariance_ = Traits::identityCovariance();
    ProcessModel processModel_{};

   private:
    void propagateCovariance(double dt, const StateVector& linearizationPoint, const StateMatrix& processNoise) {
        using HasJacobianTag = std::integral_constant<bool, ProcessModel::supportsAnalyticJacobian()>;
        propagateCovarianceImpl(dt, linearizationPoint, processNoise, HasJacobianTag{});
    }

    void propagateCovarianceImpl(double dt,
                                 const StateVector& linearizationPoint,
                                 const StateMatrix& processNoise,
                                 std::true_type) {
        const StateMatrix F = processModel_.stateJacobian(dt, linearizationPoint);
        covariance_ = F * covariance_ * F.transpose() + processNoise;
    }

    void propagateCovarianceImpl(double, const StateVector&, const StateMatrix& processNoise, std::false_type) {
        covariance_ += processNoise;
    }
};

#endif  // BASILISK_STATICKALMANBASE_HPP
