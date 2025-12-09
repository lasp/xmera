/*
 ISC License

 Copyright (c) 2025, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

 Permission to use, copy, modify, and/or distribute this software for any
 purpose with or without fee is hereby granted, provided that the above
 copyright notice and this permission notice appear in all copies.

 THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
 WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
 MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
 ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
 WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
 ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
 OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.

 */

#ifndef RW_MOTOR_TORQUE_ALGORITHM_H
#define RW_MOTOR_TORQUE_ALGORITHM_H

#include <stdint.h>

#include <architecture/msgPayloadDef/CmdTorqueBodyMsgPayload.h>
#include <architecture/msgPayloadDef/RWArrayConfigMsgPayload.h>
#include <architecture/msgPayloadDef/RWAvailabilityMsgPayload.h>
#include <architecture/msgPayloadDef/RwMotorTorqueMsgPayload.h>

#include <Eigen/Core>

/*! @brief Top level structure for the sub-module routines. */
class RwMotorTorqueAlgorithm {
   public:
    void reset(RWArrayConfigMsgPayload& rwParamsInMsg, bool rwAvailIsLinked);
    RwMotorTorqueMsgPayload update(CmdTorqueBodyMsgPayload& LrInputMsg,
                                   CmdTorqueBodyMsgPayload& LrInput2Msg,
                                   RWAvailabilityMsgPayload& wheelsAvailability,
                                   bool cmdTorque2IsLinked,
                                   bool rwAvailIsLinked);

    void setControlAxes(const Eigen::Matrix3d& controlMappingMatrix);
    Eigen::Matrix3d getControlAxes() const;

   private:
    Eigen::Matrix3d controlAxes_B{Eigen::Matrix3d::Zero()};  //!< [-] array of the control unit axes
    uint32_t numControlAxes{};  //!< [-] counter indicating how many orthogonal axes are controlled
    uint32_t numAvailRW{};      //!< [-] number of reaction wheels available
    RWArrayConfigMsgPayload
        rwConfigParams{};  //!< [-] struct to store message containing RW config parameters in body B frame
    Eigen::Matrix<double, 3, RW_EFF_CNT> G_s_B{};  //!< [-] The RW spin axis matrix in body frame components
};

#endif
