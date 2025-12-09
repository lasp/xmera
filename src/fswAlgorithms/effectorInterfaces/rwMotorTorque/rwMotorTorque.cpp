/*
 Mapping required attitude control torque Lr to RW motor torques

 */

#include "rwMotorTorque.h"

#include <stdexcept>

/*! This method performs a complete reset of the module.  Local module variables that retain
 time varying states between function calls are reset to their default values.
 @return void
 @param callTime The clock time at which the function was called (nanoseconds)
 */
void RwMotorTorque::reset(uint64_t callTime) {
    // check if the required input messages are included
    if (!this->rwParamsInMsg.isLinked()) {
        throw std::invalid_argument("rwMotorTorque.rwParamsInMsg wasn't connected.");
    }
    if (!this->vehControlInMsg.isLinked()) {
        throw std::invalid_argument("rwMotorTorque.vehControlInMsg wasn't connected.");
    }

    RWArrayConfigMsgPayload rwParams = this->rwParamsInMsg();
    bool rwAvailIsLinked = this->rwAvailInMsg.isLinked();

    this->algorithm.reset(rwParams, rwAvailIsLinked);
}

/*! Computes the reaction wheel torques given a commanded torque on the spacecraft
 @return void
 @param callTime The clock time at which the function was called (nanoseconds)
 */
void RwMotorTorque::updateState(uint64_t callTime) {
    CmdTorqueBodyMsgPayload LrInputMsg = this->vehControlInMsg(); /*!< Msg containing Lr control torque */
    CmdTorqueBodyMsgPayload LrInput2Msg{};                        /*!< Msg containing optional Lr control torque */
    RWAvailabilityMsgPayload wheelsAvailability{};                /*!< Msg containing RW availability */
    bool cmdTorque2IsLinked{};
    bool rwAvailIsLinked{};

    /*! - Check if the optional second message is provided */
    if (this->vehControlIn2Msg.isLinked()) {
        LrInput2Msg = this->vehControlIn2Msg();
        cmdTorque2IsLinked = true;
    }

    /*! - Check if RW availability message is available */
    if (this->rwAvailInMsg.isLinked()) {
        wheelsAvailability = this->rwAvailInMsg();
        rwAvailIsLinked = true;
    }

    RwMotorTorqueMsgPayload rwMotorTorques =
        algorithm.update(LrInputMsg, LrInput2Msg, wheelsAvailability, cmdTorque2IsLinked, rwAvailIsLinked);

    this->rwMotorTorqueOutMsg.write(&rwMotorTorques, this->moduleID, callTime);
}

/*! Setter method for the control axes mapping matrix CB, where each row includes the transpose of a control axis.
 The matrix needs to be 3x3, so if only 2 axes are controlled, the third row should be all zeros.
 @return void
 @param controlMappingMatrix Known external torque expressed in body frame components
*/
void RwMotorTorque::setControlAxes(const Eigen::Matrix3d& controlMappingMatrix) {
    this->algorithm.setControlAxes(controlMappingMatrix);
}

/*! Getter method for the control axes mapping matrix CB.
 @return const Eigen::Matrix3d
*/
Eigen::Matrix3d RwMotorTorque::getControlAxes() const { return this->algorithm.getControlAxes(); }
