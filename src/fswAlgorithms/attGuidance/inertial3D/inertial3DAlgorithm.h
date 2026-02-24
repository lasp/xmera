// SPDX-License-Identifier: ISC
// Copyright (c) 2025, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

#ifndef INERTIAL3DALGORITHM_H
#define INERTIAL3DALGORITHM_H

#include <architecture/msgPayloadDef/AttRefMsgPayload.h>
#include <Eigen/Core>

/*!@brief Data structure for module to compute the Inertial-3D pointing navigation solution.
 */
class Inertial3DAlgorithm {
   public:
    AttRefMsgPayload update();
    void setSigmaR0N(const Eigen::Vector3d& sigma_RN);
    const Eigen::Vector3d& getSigmaR0N() const;

   private:
    Eigen::Vector3d sigma_R0N{Eigen::Vector3d::Zero()};  //!<  MRP from inertial frame N to corrected reference frame R
};

#endif
