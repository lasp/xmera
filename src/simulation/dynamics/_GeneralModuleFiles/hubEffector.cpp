// SPDX-License-Identifier: ISC
// Copyright (c) 2016, Autonomous Vehicle System Lab, University of Colorado at Boulder
// Copyright (c) 2025, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

#include "hubEffector.h"

#include <architecture/utilities/eigenSupport.h>

/*! This is the constructor, setting variables to default values */
HubEffector::HubEffector() {
    // - zero certain variables
    this->MRPSwitchCount = 0;
    this->r_CN_NInit.setZero();
    this->v_CN_NInit.setZero();
    this->sigma_BNInit.setZero();
    this->omega_BN_BInit.setZero();

    // - define default names for the hub states
    this->nameOfHubPosition = "hubPosition";
    this->nameOfHubVelocity = "hubVelocity";
    this->nameOfHubSigma = "hubSigma";
    this->nameOfHubOmega = "hubOmega";
    this->nameOfHubNonGravVelocity = "hubNonGravVelocity";

    // - define a default mass of 1kg
    this->mHub = 1.0;
    this->IHubPntBc_B.setIdentity();
    this->r_BcB_B.fill(0.0);

    return;
}

/*! This is the destructor, nothing to report here */
HubEffector::~HubEffector() {
    return;
}

/*! The hub needs no states or properties from the other effectors */
void HubEffector::linkInStates(DynParamManager &statesIn) {
    return;
}

void HubEffector::prependSpacecraftNameToStates() {
    this->nameOfHubPosition = this->nameOfSpacecraftAttachedTo + this->nameOfHubPosition;
    this->nameOfHubVelocity = this->nameOfSpacecraftAttachedTo + this->nameOfHubVelocity;
    this->nameOfHubSigma = this->nameOfSpacecraftAttachedTo + this->nameOfHubSigma;
    this->nameOfHubOmega = this->nameOfSpacecraftAttachedTo + this->nameOfHubOmega;
    this->nameOfHubNonGravVelocity = this->nameOfSpacecraftAttachedTo + this->nameOfHubNonGravVelocity;

    return;
}

/*! This method allows the hub to register its states: r_BN_N, v_BN_N, sigma_BN and omega_BN_B */
void HubEffector::registerStates(DynParamManager &states) {
    // - Register the hub states and set with initial values
    this->posState = states.registerState(3, 1, this->nameOfHubPosition);
    this->velocityState = states.registerState(3, 1, this->nameOfHubVelocity);
    this->sigmaState = states.registerState(3, 1, this->nameOfHubSigma);
    this->omegaState = states.registerState(3, 1, this->nameOfHubOmega);
    this->nonGravVelocityState = states.registerState(3, 1, this->nameOfHubNonGravVelocity);
    /* - r_BN_N and v_BN_N of the hub is first set to r_CN_N and v_CN_N and then is corrected in spacecraft
     initializeDynamics to incorporate the fact that point B and point C are not necessarily coincident */
    this->posState->setState(this->r_CN_NInit);
    this->velocityState->setState(this->v_CN_NInit);
    this->sigmaState->setState(this->sigma_BNInit);
    this->omegaState->setState(this->omega_BN_BInit);
    this->nonGravVelocityState->setState(Eigen::Vector3d::Zero());

    return;
}

/*! This method allows the hub to give its mass properties to the spacecraft */
void HubEffector::updateEffectorMassProps(double integTime) {
    // - Give the mass to mass props
    this->effProps.mEff = this->mHub;

    // - Provide information about multi-spacecraft origin if needed
    this->r_BcP_P = this->r_BP_P + this->dcm_BP.transpose() * (this->r_BcB_B);
    this->IHubPntBc_P = this->dcm_BP.transpose() * this->IHubPntBc_B * this->dcm_BP;

    // - Give inertia of hub about point B to mass props
    this->effProps.IEffPntB_B =
        this->IHubPntBc_P + this->mHub * eigenTilde(this->r_BcP_P) * eigenTilde(this->r_BcP_P).transpose();

    // - Give position of center of mass of hub with respect to point B to mass props
    this->effProps.rEff_CB_B = this->r_BcP_P;

    // - Zero body derivatives for position and inertia;
    this->effProps.rEffPrime_CB_B.setZero();
    this->effProps.IEffPrimePntB_B.setZero();

    return;
}

/*! This method is for computing the derivatives of the hub: rDDotNonGrav_BN_N and omegaDot_BN_B, along with the
 kinematic derivatives. The spacecraft adds gravity to the velocity derivative afterwards */
void HubEffector::computeDerivatives(
    double integTime,
    Eigen::Vector3d rDDotNonGrav_BN_N,
    Eigen::Vector3d omegaDot_BN_B,
    Eigen::Vector3d sigma_BN
) {
    // - Get variables from state manager
    Eigen::Vector3d rDotLocal_BN_N;
    Eigen::MRPd sigmaLocal_BN;
    Eigen::Vector3d omegaLocal_BN_B;
    Eigen::Vector3d cLocal_B;
    Eigen::Vector3d cPrimeLocal_B;
    rDotLocal_BN_N = velocityState->getState();
    sigmaLocal_BN = (Eigen::Vector3d) sigmaState->getState();
    omegaLocal_BN_B = omegaState->getState();

    // - Set kinematic derivative
    sigmaState->setDerivative(1.0 / 4.0 * sigmaLocal_BN.Bmat() * omegaLocal_BN_B);

    // - Define dcm's
    Eigen::Matrix3d dcm_NB;
    Eigen::Matrix3d dcm_BN;
    dcm_NB = sigmaLocal_BN.toRotationMatrix();
    dcm_BN = dcm_NB.transpose();

    // - Solve for omegaDot_BN_B
    Eigen::Vector3d omegaDotLocal_BN_B;
    Eigen::Matrix3d intermediateMatrix;
    Eigen::Vector3d intermediateVector;
    intermediateVector = this->hubBackSubMatrices.vecRot
                       - this->hubBackSubMatrices.matrixC * this->hubBackSubMatrices.matrixA.inverse()
                             * this->hubBackSubMatrices.vecTrans;
    intermediateMatrix = hubBackSubMatrices.matrixD
                       - hubBackSubMatrices.matrixC * hubBackSubMatrices.matrixA.inverse() * hubBackSubMatrices.matrixB;
    omegaDotLocal_BN_B = intermediateMatrix.inverse() * intermediateVector;
    omegaState->setDerivative(omegaDotLocal_BN_B);

    // - Solve for rDDotNonGrav_BN_N
    Eigen::Vector3d rDDotNonGravLocal_BN_N =
        dcm_NB * hubBackSubMatrices.matrixA.inverse()
        * (hubBackSubMatrices.vecTrans - hubBackSubMatrices.matrixB * omegaDotLocal_BN_B);
    velocityState->setDerivative(rDDotNonGravLocal_BN_N);
    nonGravVelocityState->setDerivative(rDDotNonGravLocal_BN_N);

    // - Set kinematic derivative
    posState->setDerivative(rDotLocal_BN_N);

    return;
}

/*! This method is for computing the energy and momentum contributions from the hub */
void HubEffector::updateEnergyMomContributions(
    double integTime,
    Eigen::Vector3d &rotAngMomPntCContr_B,
    double &rotEnergyContr,
    Eigen::Vector3d omega_BN_B
) {
    // - Get variables needed for energy momentum calcs
    Eigen::Vector3d omegaLocal_BN_B;
    omegaLocal_BN_B = omegaState->getState();

    //  - Find rotational angular momentum contribution from hub
    Eigen::Vector3d rDot_BcB_B;
    rDot_BcB_B = omegaLocal_BN_B.cross(r_BcP_P);
    rotAngMomPntCContr_B = IHubPntBc_P * omegaLocal_BN_B + mHub * r_BcP_P.cross(rDot_BcB_B);

    // - Find rotational energy contribution from the hub
    rotEnergyContr =
        1.0 / 2.0 * omegaLocal_BN_B.dot(IHubPntBc_P * omegaLocal_BN_B) + 1.0 / 2.0 * mHub * rDot_BcB_B.dot(rDot_BcB_B);

    return;
}

/*! This method is for switching the MRPs */
void HubEffector::modifyStates(double integTime) {
    // Lets switch those MRPs!!
    Eigen::Vector3d sigmaBNLoc;
    sigmaBNLoc = (Eigen::Vector3d) this->sigmaState->getState();
    if (sigmaBNLoc.norm() > 1) {
        sigmaBNLoc = -sigmaBNLoc / (sigmaBNLoc.dot(sigmaBNLoc));
        this->sigmaState->setState(sigmaBNLoc);
        this->MRPSwitchCount++;
    }
    return;
}

/*! This method zeros the non-gravitational velocity state, so that after an integration step it holds the velocity
 change of point B from non-gravitational acceleration over that step */
void HubEffector::resetNonGravVelocityState() {
    this->nonGravVelocityState->setState(Eigen::Vector3d::Zero());
}
