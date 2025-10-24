/*
 Mapping required attitude control torque Lr to RW motor torques

 */

#include "rwMotorTorque.h"
#include <architecture/utilities/eigenSupport.h>

#include <stdexcept>

/*! This method performs a complete reset of the module.  Local module variables that retain
 time varying states between function calls are reset to their default values.
 @return void
 @param callTime The clock time at which the function was called (nanoseconds)
 */
void RwMotorTorque::reset(uint64_t callTime) {
    /*!- configure the number of axes that are controlled.
     This is determined by checking for a zero row to determinate search */
    this->numControlAxes = 0;
    for (uint32_t i = 0; i < 3; ++i) {
        if (this->controlAxes_B.row(i).norm() > 0.0) {
            if (this->numControlAxes < i) {
                throw std::invalid_argument("rwMotorTorque: found empty control axis. "
                                            "Make sure to fill controlAxes matrix from top to bottom, "
                                            "with zero axes (no control) at the bottom.");
            }
            this->numControlAxes += 1;
        }
    }
    if (this->numControlAxes == 0) {
        throw std::invalid_argument("rwMotorTorque is not setup to control any axes.");
    }

    // check if the required input messages are included
    if (!this->rwParamsInMsg.isLinked()) {
        throw std::invalid_argument("rwMotorTorque.rwParamsInMsg wasn't connected.");
    }
    if (!this->vehControlInMsg.isLinked()) {
        throw std::invalid_argument("rwMotorTorque.vehControlInMsg wasn't connected.");
    }

    /*! - Read static RW config data message and store it in module variables */
    this->rwConfigParams = this->rwParamsInMsg();

    /*! - If no info is provided about RW availability we'll assume that all are available
     and create the [Gs] projection matrix once */
    if (!this->rwAvailInMsg.isLinked()) {
        this->numAvailRW = this->rwConfigParams.numRW;
        this->G_s_B = cArrayAsEigenMatrix<double, 3, RW_EFF_CNT>(this->rwConfigParams.GsMatrix_B);
    }
}

/*! Add a description of what this main Update() routine does for this module
 @return void
 @param callTime The clock time at which the function was called (nanoseconds)
 */
void RwMotorTorque::updateState(uint64_t callTime) {
    RWAvailabilityMsgPayload wheelsAvailability{}; /*!< Msg containing RW availability */
    // wheelAvailability set to 0 (AVAILABLE) by default

    /*! - zero control torque and RW motor torque variables */
    Eigen::Vector<double, RW_EFF_CNT> us = Eigen::Vector<double, RW_EFF_CNT>::Zero();

    /*! - Read the input messages */
    CmdTorqueBodyMsgPayload LrInputMsg = this->vehControlInMsg();  /*!< Msg containing Lr control torque */
    Eigen::Vector3d Lr_B = cArrayAsEigenVector(LrInputMsg.torqueRequestBody);

    /*! - Check if the optional second message is provided */
    if (this->vehControlIn2Msg.isLinked()) {
        CmdTorqueBodyMsgPayload LrInput2Msg = this->vehControlIn2Msg();  /*!< Msg containing optional Lr control torque */
        Lr_B += cArrayAsEigenVector(LrInput2Msg.torqueRequestBody);
    }

    /*! - Check if RW availability message is available */
    if (this->rwAvailInMsg.isLinked()) {
        uint32_t numAvailWheels = 0;
        this->G_s_B.setZero();

        /*! - Read in current RW availabilit Msg */
        wheelsAvailability = this->rwAvailInMsg();
        /*! - create the current [Gs] projection matrix with the available RWs */
        for (uint32_t i = 0; i < this->rwConfigParams.numRW; ++i) {
            if (wheelsAvailability.wheelAvailability[i] == AVAILABLE) {
                this->G_s_B.col(numAvailWheels) = cArrayAsEigenVector3(&this->rwConfigParams.GsMatrix_B[i * 3]);
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
        us_avail.topRows(numCols) = CGs.topLeftCorner(numRows, numCols).transpose() * (CGs.topLeftCorner(numRows, numCols) * CGs.topLeftCorner(numRows, numCols).transpose()).inverse() * Lr_C.topRows(numRows);

        /*! - map the desired RW motor torques to the available RWs */
        uint32_t j = 0;
        for (uint32_t i = 0; i < this->rwConfigParams.numRW; ++i) {
            if (wheelsAvailability.wheelAvailability[i] == AVAILABLE) {
                us[i] = us_avail[j];
                j += 1;
            }
        }
    }

    /* store the output message */
    RwMotorTorqueMsgPayload rwMotorTorques{};
    eigenVectorToCArray(us, rwMotorTorques.motorTorque);

    this->rwMotorTorqueOutMsg.write(&rwMotorTorques, this->moduleID, callTime);
}

/*! Setter method for the control axes mapping matrix CB, where each row includes the transpose of a control axis.
 The matrix needs to be 3x3, so if only 2 axes are controlled, the third row should be all zeros.
 @return void
 @param controlMappingMatrix Known external torque expressed in body frame components
*/
void RwMotorTorque::setControlAxes(const Eigen::Matrix3d& controlMappingMatrix) {
    this->controlAxes_B = controlMappingMatrix;
}

/*! Getter method for the control axes mapping matrix CB.
 @return const Eigen::Matrix3d
*/
Eigen::Matrix3d RwMotorTorque::getControlAxes() const { return this->controlAxes_B; }
