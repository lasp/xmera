#ifndef BASILISK_STATICMEASUREMENTMODEL_HPP
#define BASILISK_STATICMEASUREMENTMODEL_HPP

#include "StaticFilterTraits.hpp"

#include <type_traits>

/**
 * @brief CRTP helper that standardises the interface of a measurement model.
 *
 * Concrete measurement models should inherit from MeasurementModelBase and implement:
 *   - MeasurementVector predictImpl(const StateVector& x) const;
 *   - MeasurementCovariance noiseCovarianceImpl() const;
 *   - (optional) MeasurementJacobian jacobianImpl(const StateVector& x) const;
 */
template <typename TraitsT, int MeasDimT, typename DerivedT>
class MeasurementModelBase {
   public:
    using Traits = TraitsT;
    using Derived = DerivedT;

    static constexpr int measurement_dim = MeasDimT;

    using MeasurementTraits = typename Traits::template MeasurementTraits<measurement_dim>;
    using StateVector = typename Traits::StateVector;
    using MeasurementVector = typename MeasurementTraits::Vector;
    using MeasurementCovariance = typename MeasurementTraits::Covariance;
    using MeasurementJacobian = typename MeasurementTraits::Jacobian;

    MeasurementVector predict(const StateVector& x) const { return asDerived().predictImpl(x); }

    MeasurementCovariance noiseCovariance() const { return asDerived().noiseCovarianceImpl(); }

    template <typename D = Derived, typename = std::enable_if_t<HasJacobian<D>::value>>
    MeasurementJacobian jacobian(const StateVector& x) const {
        return asDerived().jacobianImpl(x);
    }

    static constexpr bool supportsAnalyticJacobian() { return HasJacobian<Derived>::value; }

   protected:
    MeasurementModelBase() = default;
    ~MeasurementModelBase() = default;

   private:
    template <typename Candidate, typename = void>
    struct HasJacobian : std::false_type {};

    template <typename Candidate>
    struct HasJacobian<
        Candidate,
        std::void_t<decltype(std::declval<const Candidate&>().jacobianImpl(std::declval<StateVector>()))>>
        : std::true_type {};

    const Derived& asDerived() const { return static_cast<const Derived&>(*this); }
};

#endif  // BASILISK_STATICMEASUREMENTMODEL_HPP
