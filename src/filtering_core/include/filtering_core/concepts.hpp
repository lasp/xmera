// SPDX-License-Identifier: ISC
// Copyright (c) 2026, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

#ifndef FILTERING_CORE_CONCEPTS_HPP
#define FILTERING_CORE_CONCEPTS_HPP

#include <Eigen/Core>
#include <concepts>
#include <type_traits>

namespace filtering {

// A type can be linearly combined: it has scale(k) and add(other) returning
// itself. Used by RK4 / propagate to integrate any state-shaped value, not
// just numeric vectors.
template<class T>
concept LinearlyCombinable = requires(T const t, double k) {
    { t.scale(k) } -> std::same_as<T>;
    { t.add(t)   } -> std::same_as<T>;
};

// Filter state: linearly-combinable, with a static size and a raw() accessor
// returning a const reference to its underlying Eigen vector.
template<class S>
concept FilterState = LinearlyCombinable<S> && requires(S const s) {
    { S::size } -> std::convertible_to<int>;
    { s.raw()  } -> std::convertible_to<typename S::Storage const&>;
};

// A measurement model exposes:
//   - a static `size` (the dimension of the observation vector),
//   - the actual observation,
//   - the predicted observation given a state (the measurement function h(x)),
//   - the noise covariance,
//   - a subtraction operation (typically observed - predicted, but can be
//     non-trivial for circular / MRP-shadow / etc. quantities).
template<class M, class State>
concept Measurement = requires(M const m, State const s) {
    { M::size }        -> std::convertible_to<int>;
    { m.observation() } -> std::convertible_to<Eigen::Vector<double, M::size>>;
    { m.model(s)      } -> std::convertible_to<Eigen::Vector<double, M::size>>;
    { m.noise()       } -> std::convertible_to<Eigen::Matrix<double, M::size, M::size>>;
    { m.subtract(m.observation(), m.observation()) }
                       -> std::convertible_to<Eigen::Vector<double, M::size>>;
};

// A dynamics function maps (time, state) -> state. Anything callable with
// that signature qualifies — a functor, a function pointer, or a lambda
// (stored by value as the SRUKF's concrete dynamics type; not type-erased).
template<class D, class State>
concept Dynamics = std::invocable<D, double, State>
                && std::same_as<std::invoke_result_t<D, double, State>, State>;

}  // namespace filtering

#endif
