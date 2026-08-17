// SPDX-License-Identifier: ISC
// Copyright (c) 2016, Autonomous Vehicle System Lab, University of Colorado at Boulder
// Copyright (c) 2025, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

#ifndef DYNAMIC_EFFECTOR_H
#define DYNAMIC_EFFECTOR_H

#include "dynParamManager.h"

#include <Eigen/Dense>

/*! @brief Base class for an effector that applies a force or a torque to a dynamicObject but holds no
 * states for the integrator.
 *
 * Examples are thrusters, atmospheric drag and solar radiation pressure.
 *
 * The dynamicObject calls computeForceTorque() on each of its effectors and then adds forceExternal_N,
 * forceExternal_B and torqueExternalPntB_B of all of them. The effector must therefore write its full
 * contribution at each call, because the dynamicObject does not clear these values between steps.
 */
class DynamicEffector {
public:
    Eigen::VectorXd stateDerivContribution;  //!< Contribution of this effector to the state derivative of a state
                                             //!< effector. A thruster gives its mass flow rate to its fuel tank.
    Eigen::Vector3d forceExternal_N =
        Eigen::Vector3d::Zero();  //!< [N] Force that the effector applies to the spacecraft, in N frame components
    Eigen::Vector3d forceExternal_B =
        Eigen::Vector3d::Zero();  //!< [N] Force that the effector applies to the spacecraft, in B frame components
    Eigen::Vector3d torqueExternalPntB_B =
        Eigen::Vector3d::Zero();  //!< [N-m] Torque that the effector applies to the spacecraft about point B, in B
                                  //!< frame components

public:
    DynamicEffector() = default;
    virtual ~DynamicEffector() = default;

    /*! @brief Gets the states that the effector needs from the state manager.
     *
     * @param[in] states State manager of the dynamicObject
     */
    virtual void linkInStates(DynParamManager &states) = 0;

    /*! @brief Calculates the force and the torque that the effector applies to the spacecraft.
     *
     * The effector writes the results to forceExternal_N, forceExternal_B and torqueExternalPntB_B.
     *
     * @param integTime [s] Integration time
     * @param timeStep [s] Integration time step
     */
    virtual void computeForceTorque(double integTime, double timeStep) = 0;

    /*! @brief Adds the contribution of the effector to the state derivative of a state effector.
     *
     * The effector writes the result to stateDerivContribution. A fuel tank uses this to collect the
     * mass flow rate of each thruster that draws from it.
     *
     * @param integTime [s] Integration time
     */
    virtual void computeStateContribution(double integTime) {}
};

#endif /* DYNAMIC_EFFECTOR_H */
