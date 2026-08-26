// SPDX-License-Identifier: ISC
// Copyright (c) 2023, Autonomous Vehicle System Lab, University of Colorado at Boulder
// Copyright (c) 2025, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

#ifndef DYNAMICOBJECT_H
#define DYNAMICOBJECT_H

#include <architecture/_GeneralModuleFiles/sys_model.h>
#include <architecture/utilities/bskLogging.h>

#include <simulation/dynamics/_GeneralModuleFiles/dynamicEffector.h>
#include <simulation/dynamics/_GeneralModuleFiles/dynParamManager.h>
#include <simulation/dynamics/_GeneralModuleFiles/stateEffector.h>
#include <simulation/dynamics/_GeneralModuleFiles/stateVecIntegrator.h>
#include <stdint.h>

#include <vector>

/** A DynamicObject is a Basilisk model with states that must be integrated */
class DynamicObject : public SysModel {
public:
    DynParamManager dynManager;     /**< Dynamics parameter manager for all effectors */
    StateVecIntegrator* integrator; /**< Integrator used to propagate state forward */
    BSKLogger bskLogger;            /**< BSK Logging */

public:
    DynamicObject() = default;
    DynamicObject(DynamicObject const &) = delete;
    DynamicObject &operator=(DynamicObject const &) = delete;
    DynamicObject(DynamicObject &&) = delete;
    DynamicObject &operator=(DynamicObject &&) = delete;
    virtual ~DynamicObject() = default;

    /** Hooks the dyn-object into Basilisk architecture */
    virtual void updateState(uint64_t callTime) = 0;

    /** Computes F = Xdot(X,t) */
    virtual void equationsOfMotion(double t, double timeStep) = 0;

    /** Performs pre-integration steps */
    virtual void preIntegration(double callTime) = 0;

    /** Performs post-integration steps */
    virtual void postIntegration(double callTime) = 0;

    /** Initializes the dynamics and variables */
    virtual void initializeDynamics() {}

    /** Computes energy and momentum of the system */
    virtual void computeEnergyMomentum(double t) {}

    /** Prepares the dynamic object to be integrated, integrates the states
     * forward in time, and finally performs the post-integration steps.
     *
     * This is only done if the DynamicObject integration is not sync'd to another DynamicObject
     */
    void integrateState(double t);

    /** Sets a new integrator in use */
    void setIntegrator(StateVecIntegrator* newIntegrator);

public:
    double timeStep;   /**< [s] integration time step */
    double timeBefore; /**< [s] prior time value */
};

#endif /* DYNAMICOBJECT_H */
