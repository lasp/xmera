// SPDX-License-Identifier: ISC
// Copyright (c) 2024, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

#include "dynamicModels.h"

/*! Runge-Kutta 4 (RK4) function for state propagation
    @param ODEfunction function handle that includes the equations of motion
    @param X0 initial state
    @param t0 initial time
    @param dt time step
    @return Eigen::VectorXd
*/
static FilterStateVector rk4(
    DynamicsModel const& ODEfunction,
    FilterStateVector const& X0,
    double t0,
    double dt
) {
    double h = dt;

    FilterStateVector k1 = ODEfunction(t0, X0);
    FilterStateVector k2 = ODEfunction(t0 + h / 2., X0.add(k1.scale(h / 2.)));
    FilterStateVector k3 = ODEfunction(t0 + h / 2., X0.add(k2.scale(h / 2.)));
    FilterStateVector k4 = ODEfunction(t0 + h, X0.add(k3.scale(h)));

    return X0.add(
        k1.scale(h/6.)
            .add(k2.scale(h/3.))
            .add(k3.scale(h/3.))
            .add(k4.scale(h/6.))
    );
}


/*! Call the propagation function for the dynamics
   @param propagator function handle that includes the equations of motion
   @param FilterStateVector state : initial state
   @param std::array<double, 2> interval : time interval
   @param double dt: time step
   @return FilterStateVector propagatedState
*/
FilterStateVector xmera::propagate(
    DynamicsModel const& propagator,
    FilterStateVector state,
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
