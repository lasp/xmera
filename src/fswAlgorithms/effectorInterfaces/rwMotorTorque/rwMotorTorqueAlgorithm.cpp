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

#include "rwMotorTorqueAlgorithm.h"
#include <architecture/utilities/eigenSupport.h>

#include <stdexcept>

/*! This method performs a complete reset of the module.  Local module variables that retain
 time varying states between function calls are reset to their default values.
 @return void
 @param rwParamsInMsg struct to store message containing RW config parameters
 @param rwAvailIsLinked boolean indicating whether RWAvailabilityMsg is linked
 */
void RwMotorTorqueAlgorithm::reset(RWArrayConfigMsgPayload& rwParamsInMsg, bool rwAvailIsLinked) {
    /*!- configure the number of axes that are controlled.
     This is determined by checking for a zero row to determinate search */
    this->numControlAxes = 0;
    for (uint32_t i = 0; i < 3; ++i) {
        if (this->controlAxes_B.row(i).norm() > 0.0) {
            if (this->numControlAxes < i) {
                throw std::invalid_argument(
                    "rwMotorTorque: found empty control axis. "
                    "Make sure to fill controlAxes matrix from top to bottom, "
                    "with zero axes (no control) at the bottom.");
            }
            this->numControlAxes += 1;
        }
    }
    if (this->numControlAxes == 0) {
        throw std::invalid_argument("rwMotorTorque is not setup to control any axes.");
    }

    /*! - Read static RW config data message and store it in module variables */
    this->rwConfigParams = rwParamsInMsg;

    /*! - If no info is provided about RW availability we'll assume that all are available
     and create the [Gs] projection matrix once */
    if (!rwAvailIsLinked) {
        this->numAvailRW = this->rwConfigParams.numRW;
        this->G_s_B = cArrayToEigenMatrix<double, 3, RW_EFF_CNT>(this->rwConfigParams.GsMatrix_B);
    }
}

/*! Computes the reaction wheel torques given a commanded torque on the spacecraft
 @return RwMotorTorqueMsgPayload
 @param LrInputMsg commanded torque on spacecraft
 @param LrInput2Msg second commanded torque on spacecraft
 @param wheelsAvailability availability of reaction wheels
 @param cmdTorque2IsLinked boolean indicating whether a second CmdTorqueBodyMsg is linked
 @param rwAvailIsLinked boolean indicating whether RWAvailabilityMsg is linked
 */
RwMotorTorqueMsgPayload RwMotorTorqueAlgorithm::update(CmdTorqueBodyMsgPayload& LrInputMsg,
                                                       CmdTorqueBodyMsgPayload& LrInput2Msg,
                                                       RWAvailabilityMsgPayload& wheelsAvailability,
                                                       bool cmdTorque2IsLinked,
                                                       bool rwAvailIsLinked) {
    // wheelAvailability set to 0 (AVAILABLE) by default

    /*! - zero control torque and RW motor torque variables */
    Eigen::Vector<double, RW_EFF_CNT> us = Eigen::Vector<double, RW_EFF_CNT>::Zero();

    Eigen::Vector3d Lr_B = cArrayToEigenVector(LrInputMsg.torqueRequestBody);

    /*! - Check if the optional second message is provided */
    if (cmdTorque2IsLinked) {
        Lr_B += cArrayToEigenVector(LrInput2Msg.torqueRequestBody);
    }

    /*! - Check if RW availability message is available */
    if (rwAvailIsLinked) {
        uint32_t numAvailWheels = 0;
        this->G_s_B.setZero();

        /*! - create the current [Gs] projection matrix with the available RWs */
        for (uint32_t i = 0; i < this->rwConfigParams.numRW; ++i) {
            if (wheelsAvailability.wheelAvailability[i] == AVAILABLE) {
                this->G_s_B.col(numAvailWheels) = cArrayToEigenVector3(&this->rwConfigParams.GsMatrix_B[i * 3]);
                numAvailWheels += 1;
            }
        }
        /*! - update the number of currently available RWs */
        this->numAvailRW = numAvailWheels;
    }

    /*! - Compute minimum norm inverse for us = [CGs].T inv([CGs][CGs].T) [Lr_C]
     Having at least the same # of RW as # of control axes is necessary condition to guarantee inverse matrix exists. If
     matrix to invert it not full rank, the control torque output is zero. */
    if (this->numAvailRW >= this->numControlAxes) {
        uint32_t numRows = this->numControlAxes;
        uint32_t numCols = this->numAvailRW;

        Eigen::Vector3d Lr_C{Eigen::Vector3d::Zero()};
        Lr_C.head(numRows) = -this->controlAxes_B.topRows(numRows) * Lr_B;

        Eigen::Matrix<double, 3, RW_EFF_CNT> CGs = this->controlAxes_B * this->G_s_B;

        Eigen::Vector<double, RW_EFF_CNT> us_avail{Eigen::Vector<double, RW_EFF_CNT>::Zero()};
        us_avail.topRows(numCols) =
            CGs.topLeftCorner(numRows, numCols).transpose() *
            (CGs.topLeftCorner(numRows, numCols) * CGs.topLeftCorner(numRows, numCols).transpose()).inverse() *
            Lr_C.topRows(numRows);

        /*! - map the desired RW motor torques to the available RWs */
        uint32_t j = 0;
        for (uint32_t i = 0; i < this->rwConfigParams.numRW; ++i) {
            if (wheelsAvailability.wheelAvailability[i] == AVAILABLE) {
                us[i] = us_avail[j];
                j += 1;
            }
        }
    }

    RwMotorTorqueMsgPayload rwMotorTorques{};
    eigenVectorToCArray(us, rwMotorTorques.motorTorque);

    return rwMotorTorques;
}

/*! Setter method for the control axes mapping matrix CB, where each row includes the transpose of a control axis.
 The matrix needs to be 3x3, so if only 2 axes are controlled, the third row should be all zeros.
 @return void
 @param controlMappingMatrix Known external torque expressed in body frame components
*/
void RwMotorTorqueAlgorithm::setControlAxes(const Eigen::Matrix3d& controlMappingMatrix) {
    this->controlAxes_B = controlMappingMatrix;
}

/*! Getter method for the control axes mapping matrix CB.
 @return const Eigen::Matrix3d
*/
Eigen::Matrix3d RwMotorTorqueAlgorithm::getControlAxes() const { return this->controlAxes_B; }
