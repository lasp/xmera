// SPDX-License-Identifier: ISC
// Copyright (c) 2026, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

#ifndef FILTERING_CORE_DYNAMICS_MODEL_HPP
#define FILTERING_CORE_DYNAMICS_MODEL_HPP

#include <filtering_core/concepts.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <functional>

namespace filtering {

// Convenient alias for "anything that maps (time, state) -> state".
// Concrete filters typically pass static functions or stateless lambdas
// matching this signature; the Dynamics<D, State> concept (in concepts.hpp)
// is the more general gate for what predict() will accept.
template<class State>
using DynamicsModel = std::function<State(double time, State const& state)>;

// Runge-Kutta 4 step. Evaluates `dynamics` at four points per step and
// linearly combines them, hence the LinearlyCombinable constraint on State.
template<LinearlyCombinable State, Dynamics<State> D>
constexpr State rk4(D const& dynamics, State const& X0, double t0, double dt) {
    auto k1 = dynamics(t0, X0);
    auto k2 = dynamics(t0 + dt / 2., X0.add(k1.scale(dt / 2.)));
    auto k3 = dynamics(t0 + dt / 2., X0.add(k2.scale(dt / 2.)));
    auto k4 = dynamics(t0 + dt,      X0.add(k3.scale(dt)));

    return X0.add(
        k1.scale(dt / 6.)
            .add(k2.scale(dt / 3.))
            .add(k3.scale(dt / 3.))
            .add(k4.scale(dt / 6.))
    );
}

// Propagate `state` from `interval[0]` to `interval[1]` in steps of at most
// `dt`, using RK4. Returns the propagated state.
template<LinearlyCombinable State, Dynamics<State> D>
constexpr State propagate(
    D const& dynamics,
    State state,
    std::array<double, 2> interval,
    double dt
) {
    double t = interval[0];
    double const tFinal = interval[1];

    double const N = std::ceil((tFinal - t) / dt);
    for (int i = 0; i < N; i += 1) {
        double const step = std::min(dt, tFinal - t);
        state = rk4(dynamics, state, t, step);
        t += step;
    }

    return state;
}

}  // namespace filtering

#endif
