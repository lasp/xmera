// SPDX-License-Identifier: ISC
// Copyright (c) 2026, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

#ifndef FILTERING_CORE_CONCEPTS_HPP
#define FILTERING_CORE_CONCEPTS_HPP

#include <Eigen/Core>
#include <concepts>
#include <type_traits>

namespace filtering {

/*! Linear combination contract: scale(k) and add(other) return the same type. */
template<class T>
concept LinearlyCombinable = requires(T const t, double k) {
    { t.scale(k) } -> std::same_as<T>;
    { t.add(t)   } -> std::same_as<T>;
};

/*! Filter state: LinearlyCombinable, with a compile-time `size` and a `raw()` */
template<class S>
concept FilterState = LinearlyCombinable<S> && requires(S const s) {
    { S::size } -> std::convertible_to<int>;
    { s.raw()  } -> std::convertible_to<typename S::Storage const&>;
};

/*! Measurement model: exposes the observation y, the predicted observation
 *  h(x), the noise covariance R, and a subtraction. */
template<class M, class State>
concept Measurement = requires(M const m, State const s) {
    { M::size }        -> std::convertible_to<int>;
    { m.observation() } -> std::convertible_to<Eigen::Vector<double, M::size>>;
    { m.model(s)      } -> std::convertible_to<Eigen::Vector<double, M::size>>;
    { m.noise()       } -> std::convertible_to<Eigen::Matrix<double, M::size, M::size>>;
    { m.subtract(m.observation(), m.observation()) }
                       -> std::convertible_to<Eigen::Vector<double, M::size>>;
};

/*! Dynamics: callable as (time, state) -> state. Functor, function pointer,
 *  or captureless lambda all qualify — stored by value in the SRuKF. */
template<class D, class State>
concept Dynamics = std::invocable<D, double, State>
                && std::same_as<std::invoke_result_t<D, double, State>, State>;

}  // namespace filtering

#endif
