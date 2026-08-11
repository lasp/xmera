// SPDX-License-Identifier: ISC
// Copyright (c) 2016, Autonomous Vehicle System Lab, University of Colorado at Boulder
// Copyright (c) 2025, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

#ifndef STATE_EFFECTOR_H
#define STATE_EFFECTOR_H

#include "dynParamManager.h"
#include <architecture/utilities/bskLogging.h>
#include <architecture/utilities/eigenMRP.h>

#include <Eigen/Dense>

/*! back substitution matrix structure*/
struct BackSubMatrices {
    Eigen::Matrix3d matrixA;   //!< -- Back-Substitution matrix A
    Eigen::Matrix3d matrixB;   //!< -- Back-Substitution matrix B
    Eigen::Matrix3d matrixC;   //!< -- Back-Substitution matrix C
    Eigen::Matrix3d matrixD;   //!< -- Back-Substitution matrix D
    Eigen::Vector3d vecTrans;  //!< -- Back-Substitution translation vector
    Eigen::Vector3d vecRot;    //!< -- Back-Substitution rotation vector
};

/*! @brief Abstract class that is used to implement an effector attached to the dynamicObject that has a state that
 needs to be integrated. For example: reaction wheels, flexing solar panels, fuel slosh etc */
struct EffectorMassProps {
    double mEff = 0;     //!< [kg] Mass of the effector
    double mEffDot = 0;  //!< [kg/s] Time derivate of mEff
    Eigen::Matrix3d IEffPntB_B =
        Eigen::Matrix3d::Zero();  //!< [kg m^2] Inertia of effector relative to point B in B frame components
    Eigen::Vector3d rEff_CB_B =
        Eigen::Vector3d::Zero();  //!< [m] Center of mass of effector with respect to point B in B frame comp
    Eigen::Vector3d rEffPrime_CB_B =
        Eigen::Vector3d::Zero();  //!< [m/s] Time derivative with respect to the body of rEff_CB_B
    Eigen::Matrix3d IEffPrimePntB_B =
        Eigen::Matrix3d::Zero();  //!< [kg m^2/s] Time derivative with respect to the body of IEffPntB_B
};

/*! @brief state effector class */
class StateEffector {
public:
    std::string nameOfSpacecraftAttachedTo;  //!< class variable
    std::string parentSpacecraftName;        //!< -- name of the spacecraft the state effector is attached to
    EffectorMassProps effProps;              //!< -- stateEffectors instantiation of effector mass props
    Eigen::VectorXd
        stateDerivContribution;  //!< -- stateEffector contribution to another stateEffector to prevent double-counting
    Eigen::Vector3d forceOnBody_B = Eigen::Vector3d::Zero();  //!< [N] Force that the state effector applies to the s/c
    Eigen::Vector3d torqueOnBodyPntB_B =
        Eigen::Vector3d::Zero();  //!< [N] Torque that the state effector applies to the body about point B
    Eigen::Vector3d torqueOnBodyPntC_B =
        Eigen::Vector3d::Zero();  //!< [N] Torque that the state effector applies to the body about point B
    Eigen::Vector3d r_BP_P =
        Eigen::Vector3d::Zero();  //!< position vector of the spacecraft mody frame origin B relative to the primary
                                  //!< spacecraft body frame P.  This is used in the SpacecraftSystem module where
                                  //!< multiple spacecraft hubs can be a single spacecraft
    Eigen::Matrix3d dcm_BP = Eigen::Matrix3d::Identity();  //!< DCM of the spacecraft body frame B relative to primary
                                                           //!< spacecraft body frame P
    BSKLogger bskLogger;                                   //!< -- BSK Logging

public:
    StateEffector() = default;
    virtual ~StateEffector() = default;

    /*! This method is for the state effector to provide its contributions of mass and mass rates to the dynamicObject.
     * This allows for the dynamicObject to have access to the total mass, and inerita, mass and inertia rates*/
    virtual void updateEffectorMassProps(double integTime) {
    }  //!< -- Method for stateEffector to give mass contributions

    /*! This method is strictly for the back-substituion method for computing the dynamics of the spacecraft.
     * The back-sub method first computes rDDot_BN_N and omegaDot_BN_B for the spacecraft using these contributions
     * from the state effectors. Then computeDerivatives is called to compute the stateEffectors derivatives using
     * rDDot_BN_N omegaDot_BN_B*/
    virtual void updateContributions(
        double integTime,
        BackSubMatrices &backSubContr,
        Eigen::Vector3d sigma_BN,
        Eigen::Vector3d omega_BN_B,
        Eigen::Vector3d g_N
    ) {}  //!< -- Back-sub contributions

    /*! This method allows for an individual stateEffector to add its energy and momentum calculations to the
     * dynamicObject. */
    virtual void updateEnergyMomContributions(
        double integTime,
        Eigen::Vector3d &rotAngMomPntCContr_B,
        double &rotEnergyContr,
        Eigen::Vector3d omega_BN_B
    ) {}

    /*! This method allows for an individual stateEffector to modify their states after integration*/
    virtual void modifyStates(double integTime) {}

    /*! This method allows for an individual stateEffector to find the force and torque that the
     * stateEffector is placing on to the body */
    virtual void calcForceTorqueOnBody(double integTime, Eigen::Vector3d omega_BN_B) {
    }  //!< -- Force and torque on s/c due to stateEffector

    /*! This method ensures that all dynamics states have their messages written after integration */
    virtual void writeOutputStateMessages(uint64_t integTimeNanos) {}

    virtual void registerStates(DynParamManager &states) = 0;  //!< -- Method for stateEffectors to register states

    virtual void linkInStates(DynParamManager &states) = 0;  //!< -- Method for stateEffectors to get other states

    virtual void computeDerivatives(
        double integTime,
        Eigen::Vector3d rDDot_BN_N,
        Eigen::Vector3d omegaDot_BN_B,
        Eigen::Vector3d sigma_BN
    ) = 0;  //!< -- Method for each stateEffector to calculate derivatives

    /*! This method ensures that stateEffectors can be implemented using the multi-spacecraft architecture */
    virtual void prependSpacecraftNameToStates() {}
};

#endif /* STATE_EFFECTOR_H */
