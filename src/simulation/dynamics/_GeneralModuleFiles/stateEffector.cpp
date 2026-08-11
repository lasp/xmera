// SPDX-License-Identifier: ISC
// Copyright (c) 2016, Autonomous Vehicle System Lab, University of Colorado at Boulder

#include "stateEffector.h"

void StateEffector::receiveMotherSpacecraftData(Eigen::Vector3d rSC_BP_P, Eigen::Matrix3d dcmSC_BP) {
    this->r_BP_P = rSC_BP_P;
    this->dcm_BP = dcmSC_BP;
}
