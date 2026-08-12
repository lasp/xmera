// SPDX-License-Identifier: ISC
// Copyright (c) 2016, Autonomous Vehicle System Lab, University of Colorado at Boulder
// Copyright (c) 2025, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

#ifndef DYNAMIC_EFFECTOR_H
#define DYNAMIC_EFFECTOR_H

#include <architecture/utilities/bskLogging.h>

#include <simulation/dynamics/_GeneralModuleFiles/dynParamManager.h>

#include <Eigen/Dense>

/*! @brief dynamic effector class */
class DynamicEffector {
public:
    DynamicEffector();           //!< -- Constructor
    virtual ~DynamicEffector();  //!< -- Destructor
    virtual void computeStateContribution(double integTime);
    virtual void linkInStates(DynParamManager &states) = 0;  //!< -- Method to get access to other states/stateEffectors
    virtual void computeForceTorque(
        double integTime,
        double timeStep
    ) = 0;  //!< -- Method to computeForce and torque on the body

public:
    Eigen::VectorXd stateDerivContribution;  //!< -- DynamicEffectors contribution to a stateEffector
    Eigen::Vector3d forceExternal_N =
        Eigen::Vector3d::Zero();  //!< [N] External force applied by this effector in inertial components
    Eigen::Vector3d forceExternal_B =
        Eigen::Vector3d::Zero();  //!< [N] External force applied by this effector in body frame components
    Eigen::Vector3d torqueExternalPntB_B = Eigen::Vector3d::Zero();  //!< [Nm] External torque applied by this effector
    BSKLogger bskLogger;                                             //!< -- BSK Logging
};

#endif /* DYNAMIC_EFFECTOR_H */
