// SPDX-License-Identifier: ISC
// Copyright (c) 2016, Autonomous Vehicle System Lab, University of Colorado at Boulder
// Copyright (c) 2025, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

#ifndef RW_NULL_SPACE_ALGORITHM_H
#define RW_NULL_SPACE_ALGORITHM_H

#include <architecture/msgPayloadDef/RWConstellationMsgPayload.h>
#include <architecture/msgPayloadDef/RWSpeedMsgPayload.h>
#include <architecture/msgPayloadDef/RwMotorTorqueMsgPayload.h>

#include <Eigen/Core>

#include <stdint.h>
#include <stdlib.h>

/*! @brief The configuration structure for the rwNullSpace module.  */
class RwNullSpaceAlgorithm {
   public:
    void reset(RWConstellationMsgPayload& rwConfigInMsg);
    RwMotorTorqueMsgPayload update(RwMotorTorqueMsgPayload& controlRequest,
                                   RWSpeedMsgPayload& rwSpeeds,
                                   RWSpeedMsgPayload& rwDesiredSpeeds);

    void setOmegaGain(const double gain);
    double getOmegaGain() const;

   private:
    double omegaGain{};                                   //!< [-] The gain factor applied to the RW speeds
    Eigen::Matrix<double, RW_EFF_CNT, RW_EFF_CNT> tau{};  //!< [-] RW nullspace project matrix
    uint32_t numWheels{};                                 //!< [-] The number of reaction wheels we have
};

#endif
