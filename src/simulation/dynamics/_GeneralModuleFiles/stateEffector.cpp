// SPDX-License-Identifier: ISC
// Copyright (c) 2016, Autonomous Vehicle System Lab, University of Colorado at Boulder

#include "stateEffector.h"

/*! This is the constructor, just setting the variables to zero */
StateEffector::StateEffector() {
    // - set force and torques equal to zero
    this->forceOnBody_B = this->torqueOnBodyPntB_B = this->torqueOnBodyPntC_B.setZero();

    this->nameOfSpacecraftAttachedTo = "";
    this->r_BP_P.setZero();
    this->dcm_BP.setIdentity();
    return;
}

/*! This is the destructor, nothing to report here */
StateEffector::~StateEffector() {
    return;
}

/*! This method is for the state effector to provide its contributions of mass and mass rates to the dynamicObject. This
 allows for the dynamicObject to have access to the total mass, and inerita, mass and inertia rates*/
void StateEffector::updateEffectorMassProps(double integTime) {
    return;
}

void StateEffector::receiveMotherSpacecraftData(Eigen::Vector3d rSC_BP_P, Eigen::Matrix3d dcmSC_BP) {
    this->r_BP_P = rSC_BP_P;
    this->dcm_BP = dcmSC_BP;

    return;
}

/*! This method is strictly for the back-substituion method for computing the dynamics of the spacecraft. The back-sub
 method first computes rDDot_BN_N and omegaDot_BN_B for the spacecraft using these contributions from the state
 effectors. Then computeDerivatives is called to compute the stateEffectors derivatives using rDDot_BN_N omegaDot_BN_B*/
void StateEffector::updateContributions(
    double integTime,
    BackSubMatrices &backSubContr,
    Eigen::Vector3d sigma_BN,
    Eigen::Vector3d omega_BN_B,
    Eigen::Vector3d g_N
) {
    return;
}

/*! This method allows for an individual stateEffector to add its energy and momentum calculations to the dynamicObject.
 The analytical devlopement of these contributions can be seen in
 Basilisk/simulation/dynamics/_Documentation/Basilisk-EnergyAndMomentum-20161219.pdf*/
void StateEffector::updateEnergyMomContributions(
    double integTime,
    Eigen::Vector3d &rotAngMomPntCContr_B,
    double &rotEnergyContr,
    Eigen::Vector3d omega_BN_B
) {
    return;
}

/*! This method allows for an individual stateEffector to modify their states after integration*/
void StateEffector::modifyStates(double integTime) {
    return;
}

/*! This method allows for an individual stateEffector to find the force and torque that the stateEffector is placing on
 * to the body */
void StateEffector::calcForceTorqueOnBody(double integTime, Eigen::Vector3d omega_BN_B) {
    return;
}

/*! This method ensures that all dynamics states have their messages written after integation */
void StateEffector::writeOutputStateMessages(uint64_t integTimeNanos) {
    return;
}

/*! This method ensures that stateEffectors can be implemented using the multi-spacecraft archticture */
void StateEffector::prependSpacecraftNameToStates() {
    return;
}
