// SPDX-License-Identifier: ISC
// Copyright (c) 2016, Autonomous Vehicle System Lab, University of Colorado at Boulder

#include "stateEffector.h"

StateEffector::StateEffector() {
    // - set force and torques equal to zero
    this->forceOnBody_B.setZero();
    this->torqueOnBodyPntB_B.setZero();
    this->torqueOnBodyPntC_B.setZero();

    this->nameOfSpacecraftAttachedTo = "";
    this->r_BP_P.setZero();
    this->dcm_BP.setIdentity();
}

/*! This is the destructor, nothing to report here */
StateEffector::~StateEffector() {
    return;
}


void StateEffector::receiveMotherSpacecraftData(Eigen::Vector3d rSC_BP_P, Eigen::Matrix3d dcmSC_BP) {
    this->r_BP_P = rSC_BP_P;
    this->dcm_BP = dcmSC_BP;
}
