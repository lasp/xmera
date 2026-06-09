// SPDX-License-Identifier: ISC
// Copyright (c) 2026, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

#ifndef FILTERING_CORE_DYNAMICS_MODEL_HPP
#define FILTERING_CORE_DYNAMICS_MODEL_HPP

#include <filteringCore/concepts.hpp>

#include <algorithm>
#include <array>
#include <math.h>

namespace filtering {

/*! Runge-Kutta 4
 *  @return state advanced by dt
 *  @param dynamics [-] callable (t, x) -> dx/dt
 *  @param X0       [-] state at time t0
 *  @param t0       [s] start time
 *  @param dt       [s] step size */
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

/*! Fixed RK4 sub-step used by `propagate`. */
inline constexpr double kIntegrationStep = 0.2;

/*! Propagate state across a time interval in RK4 sub-steps.
 *  @return state at interval[1]
 *  @param dynamics [-]   callable (t, x) -> dx/dt
 *  @param state    [-]   state at interval[0]
 *  @param interval [s,s] {start, end} time pair */
template<LinearlyCombinable State, Dynamics<State> D>
constexpr State propagate(
    D const& dynamics,
    State state,
    std::array<double, 2> interval
) {
    double t = interval[0];
    double const tFinal = interval[1];

    double const N = ceil((tFinal - t) / kIntegrationStep);
    for (int i = 0; i < N; i += 1) {
        double const step = std::min(kIntegrationStep, tFinal - t);
        state = rk4(dynamics, state, t, step);
        t += step;
    }

    return state;
}

}  // namespace filtering

#endif
