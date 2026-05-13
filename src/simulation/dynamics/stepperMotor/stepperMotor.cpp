// SPDX-License-Identifier: ISC
// Copyright (c) 2024, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

#include "stepperMotor.h"
#include <architecture/utilities/macroDefinitions.h>
#include <cassert>
#include <cmath>

/*! Module reset method.
 @return void
 @param callTime [ns] Time the method is called
*/
void StepperMotor::reset(uint64_t callTime) {
    assert(this->motorStepCommandInMsg.isLinked());

    // Reset required module parameters
    this->theta = this->thetaInit;
    this->thetaDot = 0.0;
    this->thetaDDot = 0.0;
    this->tInit = 0.0;
    this->stepCount = 0;
    // Absolute motor position from theta=0 (so motorPosition * stepAngle ≈ theta).
    this->motorPosition = static_cast<int>(std::round(this->thetaInit / this->stepAngle));
    this->previousWrittenTime = -1;
    this->actuationComplete = true;
    this->stepComplete = true;
    this->newMsg = false;
    this->interruptMsg = false;

    // Set motor maximum angular acceleration
    this->thetaDDotMax = this->stepAngle / (0.25 * this->stepTime * this->stepTime);  // [rad/s^2]
}

/*! Module update method. This method profiles the stepper motor actuation as a function of time. The motor states
 are then written to the output message.
 @return void
 @param callTime [ns] Time the method is called
*/
void StepperMotor::updateState(uint64_t callTime) {
    MotorStepCommandMsgPayload motorStepCommandIn{};

    // Read the input message
    if (this->motorStepCommandInMsg.isWritten()) {
        motorStepCommandIn = this->motorStepCommandInMsg();
        // Store the number of commanded motor steps when a new message is written
        if (this->previousWrittenTime < this->motorStepCommandInMsg.timeWritten()) {
            this->stepsCommanded = motorStepCommandIn.stepsCommanded;
            this->previousWrittenTime = this->motorStepCommandInMsg.timeWritten();

            // Update booleans
            this->newMsg = true;
            if (this->actuationComplete) {
                this->interruptMsg = false;
            } else {
                this->interruptMsg = true;
            }
            if (this->stepsCommanded == 0) {
                this->actuationComplete = true;
                this->stepCount = 0;
            } else {
                this->actuationComplete = false;
            }
        }
    }

    // Actuate the motor only if a current actuation segment is not complete
    double t = callTime * NANO2SEC;
    bool isMotorMoving = false;
    if (!(this->actuationComplete)) {
        // Reset the motor immediately after a new non-interrupting request is received
        if ((this->newMsg && !this->interruptMsg) || (this->interruptMsg && this->stepComplete)) {
            this->resetMotor();
        }
        this->actuateMotor(t);
        isMotorMoving = true;
    } else {
        this->tInit = t;
        this->thetaDDot = 0.0;
    }

    // Write the output message
    StepperMotorMsgPayload stepperMotorOut{};
    stepperMotorOut.theta = this->theta;
    stepperMotorOut.thetaDot = this->thetaDot;
    stepperMotorOut.thetaDDot = this->thetaDDot;
    stepperMotorOut.stepsCommanded = this->stepsCommanded;
    stepperMotorOut.stepCount = this->stepCount;
    stepperMotorOut.motorPosition = this->motorPosition;
    stepperMotorOut.isMotorMoving = isMotorMoving;
    this->stepperMotorOutMsg.write(&stepperMotorOut, moduleID, callTime);
}

/*! This method is used to simulate the stepper motor actuation in time.
 @return void
 @param t [s] Time the method is called
*/
void StepperMotor::actuateMotor(double t) {
    // Update the motor step parameters when a step is completed
    if (this->stepComplete) {
        this->updateStepParameters();
    }

    // Update the motor states during each step
    if (this->isInStepFirstHalf(t)) {
        this->computeStepFirstHalf(t);
    } else if (this->isInStepSecondHalf(t)) {
        this->computeStepSecondHalf(t);
    } else {
        this->computeStepComplete(t);
    }
}

/*! This method resets the motor states when the current request is complete and a new request is received.
 @return void
*/
void StepperMotor::resetMotor() {
    this->stepCount = 0;
    this->thetaInit = this->theta;
    this->newMsg = false;
    this->interruptMsg = false;
}

/*! This method updates the step parameters after a step is completed.
 @return void
*/
void StepperMotor::updateStepParameters() {
    this->stepComplete = false;
    this->tf = this->tInit + this->stepTime;
    this->ts = this->tInit + this->stepTime / 2;
    this->intermediateThetaInit = this->thetaInit + (this->stepCount * this->stepAngle);

    if (this->stepsCommanded > 0) {
        this->intermediateThetaRef = this->thetaInit + ((this->stepCount + 1) * this->stepAngle);
        this->a = 0.5 * (this->stepAngle) / ((this->ts - this->tInit) * (this->ts - this->tInit));
        this->b = -0.5 * (this->stepAngle) / ((this->ts - this->tf) * (this->ts - this->tf));
    } else {
        this->intermediateThetaRef = this->thetaInit + ((this->stepCount - 1) * this->stepAngle);
        this->a = 0.5 * (-this->stepAngle) / ((this->ts - this->tInit) * (this->ts - this->tInit));
        this->b = -0.5 * (-this->stepAngle) / ((this->ts - this->tf) * (this->ts - this->tf));
    }
}

/*! This method determines if the motor is in the first half of a step.
 @return bool
 @param t [s] Time the method is called
*/
bool StepperMotor::isInStepFirstHalf(double t) { return (t < this->ts && std::abs(this->ts - t) > 1e-5); }

/*! This method computes the motor states during the first half of each step.
 @return void
 @param t [s] Time the method is called
*/
void StepperMotor::computeStepFirstHalf(double t) {
    if (this->intermediateThetaRef > this->intermediateThetaInit) {
        this->thetaDDot = this->thetaDDotMax;
    } else {
        this->thetaDDot = -this->thetaDDotMax;
    }
    this->thetaDot = this->thetaDDot * (t - this->tInit);
    this->theta = this->a * (t - this->tInit) * (t - this->tInit) + this->intermediateThetaInit;
}

/*! This method determines if the motor is in the second half of a step.
 @return bool
 @param t [s] Time the method is called
*/
bool StepperMotor::isInStepSecondHalf(double t) {
    return ((t >= this->ts || std::abs(this->ts - t) < 1e-5) && t < this->tf && std::abs(this->tf - t) > 1e-5);
}

/*! This method computes the motor states during the second half of each step.
 @return void
 @param t [s] Time the method is called
*/
void StepperMotor::computeStepSecondHalf(double t) {
    if (this->intermediateThetaRef > this->intermediateThetaInit) {
        this->thetaDDot = -this->thetaDDotMax;
    } else {
        this->thetaDDot = this->thetaDDotMax;
    }
    this->thetaDot = this->thetaDDot * (t - this->tInit) - this->thetaDDot * (this->tf - this->tInit);
    this->theta = this->b * (t - this->tf) * (t - this->tf) + this->intermediateThetaRef;
}

/*! This method computes the motor states when a step is complete.
 @return void
 @param t [s] Time the method is called
*/
void StepperMotor::computeStepComplete(double t) {
    this->stepComplete = true;
    this->thetaDot = 0.0;
    this->theta = this->intermediateThetaRef;
    // Anchor to tf rather than t so the (t - tf) residual carries into the next step;
    // otherwise, when the dynamics tick doesn't divide stepTime, that residual is lost
    // each step and the long-run motor rate falls below 1/stepTime.
    this->tInit = this->tf;

    // Update the motor step count
    if (this->intermediateThetaRef > this->intermediateThetaInit) {
        this->stepCount++;
        this->motorPosition++;
    } else {
        this->stepCount--;
        this->motorPosition--;
    }

    // Update the actuationComplete boolean variable only when motor actuation is complete
    if ((this->stepCount == this->stepsCommanded) && !this->interruptMsg) {
        this->actuationComplete = true;
    }
}

/*! Getter method for the initial motor angle.
 @return double
*/
double StepperMotor::getThetaInit() const { return this->thetaInit; }

/*! Getter method for the motor step angle.
 @return double
*/
double StepperMotor::getStepAngle() const { return this->stepAngle; }

/*! Getter method for the motor step time.
 @return double
*/
double StepperMotor::getStepTime() const { return this->stepTime; }

/*! Getter method for the maximum motor angular acceleration.
 @return double
*/
double StepperMotor::getThetaDDotMax() const { return this->thetaDDotMax; }

/*! Setter method for the initial motor angle.
 @return void
 @param thetaInit [rad] Initial motor angle
*/
void StepperMotor::setThetaInit(const double thetaInit) { this->thetaInit = thetaInit; }

/*! Setter method for the motor step angle.
 @return void
 @param stepAngle [rad] Motor step angle
*/
void StepperMotor::setStepAngle(const double stepAngle) {
    assert(stepAngle > 0.0);
    this->stepAngle = std::abs(stepAngle);
}

/*! Setter method for the motor step time.
 @return void
 @param stepTime [s] Motor step time
*/
void StepperMotor::setStepTime(const double stepTime) {
    assert(stepTime > 0.0);
    this->stepTime = std::abs(stepTime);
}
