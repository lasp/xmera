// SPDX-License-Identifier: ISC
// Copyright (c) 2023, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

#include "stepperMotorController.h"

#include <architecture/utilities/macroDefinitions.h>

#include <stdexcept>

/*! This method performs a complete reset of the module. The input message is checked to ensure it is linked.
 @return void
 @param callTime [ns] Time the method is called
*/
void StepperMotorController::reset(uint64_t callTime) {
    if (!this->motorRefAngleInMsg.isLinked()) {
        throw std::invalid_argument("StepperMotorController.motorRefAngleInMsg wasn't connected.");
    }

    this->algorithm.reset();
}

/*! The update method computes the required number of motor steps given a motor angle reference message. This method
 also tracks the motor actuation in time and includes logic for incoming reference commands that interrupt an unfinished
 motor actuation sequence.
 @return void
 @param callTime [ns] Time the method is called
*/
void StepperMotorController::updateState(uint64_t callTime) {
    HingedRigidBodyMsgPayload motorRefAngleIn{};
    double hingedRigidBodyMsgTimeWritten{};
    if (this->motorRefAngleInMsg.isWritten()) {
        motorRefAngleIn = this->motorRefAngleInMsg();

        // Store the time the motor reference input message was written
        hingedRigidBodyMsgTimeWritten = NANO2SEC * this->motorRefAngleInMsg.timeWritten();
    }

    StepperMotorControllerOutput stepperMotorControllerOutput =
        this->algorithm.update(callTime, hingedRigidBodyMsgTimeWritten, motorRefAngleIn);

    if (stepperMotorControllerOutput.writeOutputMessage) {
        this->motorStepCommandOutMsg.write(stepperMotorControllerOutput.motorStepCommandOut, moduleID, callTime);
    }
}

/*! Setter method for the initial motor angle.
 @return void
 @param thetaInit [rad] Initial motor angle
*/
void StepperMotorController::setThetaInit(double const thetaInit) {
    this->algorithm.setThetaInit(thetaInit);
}

/*! Getter method for the initial motor angle.
 @return double
*/
double StepperMotorController::getThetaInit() const {
    return this->algorithm.getThetaInit();
}

/*! Setter method for the motor upper actuation limit.
 @return void
 @param thetaMax [rad] Motor upper actuation limit
*/
void StepperMotorController::setThetaMax(double const thetaMax) {
    this->algorithm.setThetaMax(thetaMax);
}

/*! Getter method for the motor upper actuation limit.
 @return double
*/
double StepperMotorController::getThetaMax() const {
    return this->algorithm.getThetaMax();
}

/*! Setter method for the motor lower actuation limit.
 @return void
 @param thetaMin [rad] Motor lower actuation limit
*/
void StepperMotorController::setThetaMin(double const thetaMin) {
    this->algorithm.setThetaMin(thetaMin);
}

/*! Getter method for the motor lower actuation limit.
 @return double
*/
double StepperMotorController::getThetaMin() const {
    return this->algorithm.getThetaMin();
}

/*! Setter method for the motor step angle.
 @return void
 @param stepAngle [rad] Motor step angle
*/
void StepperMotorController::setStepAngle(double const stepAngle) {
    this->algorithm.setStepAngle(stepAngle);
}

/*! Getter method for the motor step angle.
 @return double
*/
double StepperMotorController::getStepAngle() const {
    return this->algorithm.getStepAngle();
}

/*! Setter method for the motor step time.
 @return void
 @param stepTime [s] Motor step time
*/
void StepperMotorController::setStepTime(double const stepTime) {
    this->algorithm.setStepTime(stepTime);
}

/*! Getter method for the motor step time.
 @return double
*/
double StepperMotorController::getStepTime() const {
    return this->algorithm.getStepTime();
}
