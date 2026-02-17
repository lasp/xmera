// SPDX-License-Identifier: ISC
// Copyright (c) 2025, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

#ifndef FORCE_TORQUE_THR_FORCE_MAPPING_ALGORITHM_H
#define FORCE_TORQUE_THR_FORCE_MAPPING_ALGORITHM_H

#include <cstdint>
#include <architecture/msgPayloadDef/CmdForceBodyMsgPayload.h>
#include <architecture/msgPayloadDef/CmdTorqueBodyMsgPayload.h>
#include <architecture/msgPayloadDef/THRArrayCmdForceMsgPayload.h>
#include <architecture/msgPayloadDef/THRArrayConfigMsgPayload.h>
#include <architecture/msgPayloadDef/VehicleConfigMsgPayload.h>
#include <Eigen/Core>

/*! @brief This module maps thruster forces for arbitrary forces and torques
 */
class ForceTorqueThrForceMappingAlgorithm {
   public:
    void reset(VehicleConfigMsgPayload& vehConfigMsg, THRArrayConfigMsgPayload& thrConfigMsg);
    THRArrayCmdForceMsgPayload update(CmdTorqueBodyMsgPayload& cmdTorqueMsg, CmdForceBodyMsgPayload& cmdForceMsg) const;

   private:
    uint32_t numThrusters{};  //!< []      The number of thrusters available on vehicle
    Eigen::Vector3d CoM_B{};  //!< [m]     CoM of the s/c
    Eigen::Matrix<double, 3, MAX_EFF_CNT> rThruster_B{
        Eigen::Matrix<double, 3, MAX_EFF_CNT>::Zero()};  //!< [m]     local copy of the thruster locations
    Eigen::Matrix<double, 3, MAX_EFF_CNT> gtThruster_B{
        Eigen::Matrix<double, 3, MAX_EFF_CNT>::Zero()};  //!< []      local copy of the thruster force unit direction
                                                         //!< vectors
};

#endif
