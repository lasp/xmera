// SPDX-License-Identifier: ISC
// Copyright (c) 2024, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

#ifndef FILTER_DYN_MODELS_H
#define FILTER_DYN_MODELS_H

#include <functional>

/*! @brief Measurement models used to map a state vector to a measurement */
template<typename State>
using DynamicsModel = std::function<State(double time, State const& state)>;

namespace xmera {
    template<typename State>
    concept linearly_combinable = requires(State const constState) {
        { constState.scale(std::declval<double>()) }
            -> std::same_as<State>;
        { constState.add(constState) }
            -> std::same_as<State>;
    };

    /*! Runge-Kutta 4 (RK4) function for state propagation
        @param ODEfunction function handle that includes the equations of motion
        @param X0 initial state
        @param t0 initial time
        @param dt time step
        @return Eigen::VectorXd
    */
    template<linearly_combinable State>
    static State rk4(
        DynamicsModel<State> const& ODEfunction,
        State const& X0,
        double t0,
        double dt
    ) {
        double h = dt;

        auto k1 = ODEfunction(t0, X0);
        auto k2 = ODEfunction(t0 + h/2., X0.add(k1.scale(h/2.)));
        auto k3 = ODEfunction(t0 + h/2., X0.add(k2.scale(h/2.)));
        auto k4 = ODEfunction(t0 + h, X0.add(k3.scale(h)));

        return X0.add(
            k1.scale(h/6.)
                .add(k2.scale(h/3.))
                .add(k3.scale(h/3.))
                .add(k4.scale(h/6.))
        );
    }

    template<linearly_combinable State>
    static State propagate(
        DynamicsModel<State> const& propagator,
        State state,
        std::array<double, 2> interval,
        double dt
    ) {
        double t_0 = interval[0];
        double t_f = interval[1];
        double t = t_0;

        /*! Propagate to t_final with an RK4 integrator */
        double N = ceil((t_f-t_0)/dt);
        for (int c = 0; c < N; c += 1) {
            double step = std::min(dt,t_f-t);
            state = rk4(propagator, state, t, step);
            t += step;
        }

        return state;
    }
}

#endif
