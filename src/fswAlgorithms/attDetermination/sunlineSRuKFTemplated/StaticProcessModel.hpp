#ifndef BASILISK_STATICPROCESSMODEL_HPP
#define BASILISK_STATICPROCESSMODEL_HPP

#include "StaticFilterTraits.hpp"

#include <type_traits>

/**
 * @brief CRTP helper that standardises the interface expected from a process model.
 *
 * Each concrete process model should inherit from ProcessModelBase and implement:
 *   - StateVector propagateImpl(double dt, const StateVector& x) const;
 *   - (optional) StateMatrix jacobianImpl(double dt, const StateVector& x) const;
 *
 * The base class provides small utilities (runtime detection of the Jacobian) and
 * forwards the public-facing calls to the derived implementation.
 */
template<typename TraitsT, typename DerivedT>
class ProcessModelBase {
public:
    using Traits = TraitsT;
    using Derived = DerivedT;

    using Scalar = typename Traits::Scalar;
    using StateVector = typename Traits::StateVector;
    using StateMatrix = typename Traits::StateMatrix;

    StateVector propagate(double dt, const StateVector& x) const {
        return asDerived().propagateImpl(dt, x);
    }

    template<typename D = Derived, typename = std::enable_if_t<HasJacobian<D>::value>>
    StateMatrix stateJacobian(double dt, const StateVector& x) const {
        return asDerived().jacobianImpl(dt, x);
    }

    static constexpr bool supportsAnalyticJacobian() {
        return HasJacobian<Derived>::value;
    }

protected:
    ProcessModelBase() = default;
    ~ProcessModelBase() = default;

private:
    template<typename Candidate, typename = void>
    struct HasJacobian : std::false_type {};

    template<typename Candidate>
    struct HasJacobian<Candidate, std::void_t<
        decltype(std::declval<const Candidate&>().jacobianImpl(
            std::declval<double>(), std::declval<StateVector>()))
    >> : std::true_type {};

    const Derived& asDerived() const {
        return static_cast<const Derived&>(*this);
    }
};

#endif  // BASILISK_STATICPROCESSMODEL_HPP
