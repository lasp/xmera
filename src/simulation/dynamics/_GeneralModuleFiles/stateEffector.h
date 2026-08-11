// SPDX-License-Identifier: ISC
// Copyright (c) 2016, Autonomous Vehicle System Lab, University of Colorado at Boulder
// Copyright (c) 2025, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

#ifndef STATE_EFFECTOR_H
#define STATE_EFFECTOR_H

#include "dynParamManager.h"

#include <Eigen/Dense>

/*! @brief Back-substitution contributions of one state effector.
 *
 * The dynamicObject adds the contributions of all of its effectors, then solves
 * [matrixA matrixB; matrixC matrixD] [rDDot_BN_N; omegaDot_BN_B] = [vecTrans; vecRot] for the
 * acceleration of the spacecraft hub.
 */
struct BackSubMatrices {
    Eigen::Matrix3d matrixA;   //!< [kg] Back-substitution matrix A
    Eigen::Matrix3d matrixB;   //!< [kg m] Back-substitution matrix B
    Eigen::Matrix3d matrixC;   //!< [kg m] Back-substitution matrix C
    Eigen::Matrix3d matrixD;   //!< [kg m^2] Back-substitution matrix D
    Eigen::Vector3d vecTrans;  //!< [N] Back-substitution translation vector
    Eigen::Vector3d vecRot;    //!< [N-m] Back-substitution rotation vector
};

/*! @brief Mass properties that a state effector gives to its parent spacecraft. */
struct EffectorMassProps {
    double mEff = 0;     //!< [kg] Mass of the effector
    double mEffDot = 0;  //!< [kg/s] Time derivative of mEff
    Eigen::Matrix3d IEffPntB_B =
        Eigen::Matrix3d::Zero();  //!< [kg m^2] Inertia of effector relative to point B in B frame components
    Eigen::Vector3d rEff_CB_B =
        Eigen::Vector3d::Zero();  //!< [m] Center of mass of effector with respect to point B in B frame comp
    Eigen::Vector3d rEffPrime_CB_B =
        Eigen::Vector3d::Zero();  //!< [m/s] Time derivative with respect to the body of rEff_CB_B
    Eigen::Matrix3d IEffPrimePntB_B =
        Eigen::Matrix3d::Zero();  //!< [kg m^2/s] Time derivative with respect to the body of IEffPntB_B
};

/*! @brief Base class for an effector that attaches to a dynamicObject and holds states for the integrator.
 *
 * Examples are reaction wheels, flexible solar panels and fuel slosh.
 */
class StateEffector {
public:
    std::string nameOfSpacecraftAttachedTo;  //!< [-] Name prefix of the parent spacecraft. The effector adds it in
                                             //!< front of its own state and property names.
    std::string parentSpacecraftName;        //!< [-] Name of the spacecraft that the effector attaches to
    EffectorMassProps effProps;              //!< Mass properties that this effector gives to the spacecraft
    Eigen::VectorXd stateDerivContribution;  //!< Contribution of this effector to another effector, which prevents
                                             //!< double-counting
    Eigen::Vector3d forceOnBody_B = Eigen::Vector3d::Zero();  //!< [N] Force that the effector applies to the spacecraft
    Eigen::Vector3d torqueOnBodyPntB_B =
        Eigen::Vector3d::Zero();  //!< [N-m] Torque that the effector applies to the body about point B
    Eigen::Vector3d torqueOnBodyPntC_B =
        Eigen::Vector3d::Zero();  //!< [N-m] Torque that the effector applies to the body about point C
    Eigen::Vector3d r_BP_P = Eigen::Vector3d::Zero();      //!< [m] Position of body frame origin B relative to primary
                                                           //!< body frame origin P, in P frame components
    Eigen::Matrix3d dcm_BP = Eigen::Matrix3d::Identity();  //!< [-] DCM of body frame B relative to primary body frame P

public:
    StateEffector() = default;
    virtual ~StateEffector() = default;

    /*! @brief Gives the mass and the mass rate contributions of the effector to the dynamicObject.
     *
     * The dynamicObject uses these contributions to get the total mass, the total inertia, and the
     * time derivatives of both.
     *
     * @param integTime [s] Integration time
     */
    virtual void updateEffectorMassProps(double integTime) {}

    /*! @brief Gives the back-substitution contributions of the effector.
     *
     * The back-substitution method first calculates rDDot_BN_N and omegaDot_BN_B for the spacecraft
     * from these contributions. computeDerivatives() then uses rDDot_BN_N and omegaDot_BN_B to
     * calculate the state derivatives of the effector.
     *
     * @param integTime [s] Integration time
     * @param[in,out] backSubContr Back-substitution contributions that the effector adds to
     * @param sigma_BN [-] MRP attitude of body frame B relative to inertial frame N
     * @param omega_BN_B [rad/s] Angular velocity of frame B relative to frame N, in B frame components
     * @param g_N [m/s^2] Gravitational acceleration, in N frame components
     */
    virtual void updateContributions(
        double integTime,
        BackSubMatrices &backSubContr,
        Eigen::Vector3d sigma_BN,
        Eigen::Vector3d omega_BN_B,
        Eigen::Vector3d g_N
    ) {}

    /*! @brief Adds the energy and the momentum contributions of the effector.
     *
     * @param integTime [s] Integration time
     * @param[in,out] rotAngMomPntCContr_B [kg m^2/s] Angular momentum about point C, in B frame components
     * @param[in,out] rotEnergyContr [J] Rotational energy contribution
     * @param omega_BN_B [rad/s] Angular velocity of frame B relative to frame N, in B frame components
     */
    virtual void updateEnergyMomContributions(
        double integTime,
        Eigen::Vector3d &rotAngMomPntCContr_B,
        double &rotEnergyContr,
        Eigen::Vector3d omega_BN_B
    ) {}

    /*! @brief Changes the states of the effector after integration.
     *
     * @param integTime [s] Integration time
     */
    virtual void modifyStates(double integTime) {}

    /*! @brief Calculates the force and the torque that the effector applies to the body.
     *
     * The effector writes the results to forceOnBody_B, torqueOnBodyPntB_B and torqueOnBodyPntC_B.
     *
     * @param integTime [s] Integration time
     * @param omega_BN_B [rad/s] Angular velocity of frame B relative to frame N, in B frame components
     */
    virtual void calcForceTorqueOnBody(double integTime, Eigen::Vector3d omega_BN_B) {}

    /*! @brief Writes the output messages of the effector after integration.
     *
     * @param integTimeNanos [ns] Integration time
     */
    virtual void writeOutputStateMessages(uint64_t integTimeNanos) {}

    /*! @brief Adds the states of the effector to the state manager.
     *
     * @param[in,out] states State manager of the dynamicObject
     */
    virtual void registerStates(DynParamManager &states) = 0;

    /*! @brief Gets the states that the effector needs from the state manager.
     *
     * @param[in] states State manager of the dynamicObject
     */
    virtual void linkInStates(DynParamManager &states) = 0;

    /*! @brief Calculates the state derivatives of the effector.
     *
     * @param integTime [s] Integration time
     * @param rDDot_BN_N [m/s^2] Acceleration of point B relative to frame N, in N frame components
     * @param omegaDot_BN_B [rad/s^2] Angular acceleration of frame B relative to frame N, in B frame components
     * @param sigma_BN [-] MRP attitude of body frame B relative to inertial frame N
     */
    virtual void computeDerivatives(
        double integTime,
        Eigen::Vector3d rDDot_BN_N,
        Eigen::Vector3d omegaDot_BN_B,
        Eigen::Vector3d sigma_BN
    ) = 0;

    /*! @brief Adds the name of the parent spacecraft in front of the names of the effector states.
     *
     * Each spacecraft needs its own state names when more than one spacecraft is in the simulation.
     */
    virtual void prependSpacecraftNameToStates() {}
};

#endif /* STATE_EFFECTOR_H */
