// SPDX-License-Identifier: ISC
// Copyright (c) 2025, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

#ifndef INERTIAL3D_H
#define INERTIAL3D_H

#include "inertial3DAlgorithm.h"
#include <architecture/_GeneralModuleFiles/sys_model.h>
#include <architecture/messaging/messaging.h>
#include <architecture/msgPayloadDef/AttRefMsgPayload.h>

#include <stdint.h>

#include <Eigen/Core>

/*!@brief Data structure for module to compute the Inertial-3D pointing navigation solution.
 */
class Inertial3D final : public SysModel {
public:
    void updateState(uint64_t callTime) override;
    void setSigmaR0N(Eigen::Vector3d const &sigma_RN);
    Eigen::Vector3d const &getSigmaR0N() const;

    Message<AttRefMsgPayload> attRefOutMsg;  //!< reference attitude output message

private:
    Inertial3DAlgorithm algorithm{};
};

#endif
