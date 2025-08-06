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

#include "stepperMotor.h"
#include "architecture/utilities/macroDefinitions.h"
#include <cassert>

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
    this->previousWrittenTime = -1;
    this->actuationComplete = true;
    this->stepComplete = true;
    this->newMsg = false;

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
            this->newMsg = true;
            this->stepsCommanded = motorStepCommandIn.stepsCommanded;
            this->previousWrittenTime = this->motorStepCommandInMsg.timeWritten();
            this->actuationComplete = true;
            if (this->stepsCommanded != 0) {
                this->actuationComplete = false;
            }
        }
    }

    // Actuate the motor only if a current actuation segment is not complete
    if (!(this->actuationComplete)) {
        this->actuateMotor(callTime * NANO2SEC);
    }

    // Write the output message
    StepperMotorMsgPayload stepperMotorOut{};
    stepperMotorOut.theta = this->theta;
    stepperMotorOut.thetaDot = this->thetaDot;
    stepperMotorOut.thetaDDot = this->thetaDDot;
    stepperMotorOut.stepsCommanded = this->stepsCommanded;
    stepperMotorOut.stepCount = this->stepCount;
    this->stepperMotorOutMsg.write(&stepperMotorOut, moduleID, callTime);
}

/*! This method is used to simulate the stepper motor actuation in time.
 @return void
 @param t [s] Time the method is called
*/
void StepperMotor::actuateMotor(double t) {
    // Reset the motor states when the current request is complete and a new request is received
    if (this->newMsg && this->stepComplete) {
        this->resetMotor(t);
    }

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
 @param t [s] Time the method is called
*/
void StepperMotor::resetMotor(double t) {
    this->stepCount = 0;
    this->thetaInit = this->theta;
    this->tInit = t;
    this->newMsg = false;
}

/*! This method updates the step parameters after a step is completed.
 @return void
*/
void StepperMotor::updateStepParameters() {
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
bool StepperMotor::isInStepFirstHalf(double t) {
    return (t < this->ts || t == this->ts) && (this->tf - this->tInit != 0.0);
}

/*! This method computes the motor states during the first half of each step.
 @return void
 @param t [s] Time the method is called
*/
void StepperMotor::computeStepFirstHalf(double t) {
    if (this->stepsCommanded > 0 && !this->newMsg) {
        this->thetaDDot = this->thetaDDotMax;
    } else if (!this->newMsg) {
        this->thetaDDot = -this->thetaDDotMax;
    }
    this->thetaDot = this->thetaDDot * (t - this->tInit);
    this->theta = this->a * (t - this->tInit) * (t - this->tInit) + this->intermediateThetaInit;
    this->stepComplete = false;
}

/*! This method determines if the motor is in the second half of a step.
 @return bool
 @param t [s] Time the method is called
*/
bool StepperMotor::isInStepSecondHalf(double t) {
    return (t > this->ts && t < this->tf) && (this->tf - this->tInit != 0.0);
}

/*! This method computes the motor states during the second half of each step.
 @return void
 @param t [s] Time the method is called
*/
void StepperMotor::computeStepSecondHalf(double t) {
    if (this->stepsCommanded > 0 && !this->newMsg) {
        this->thetaDDot = -this->thetaDDotMax;
    } else if (!this->newMsg) {
        this->thetaDDot = this->thetaDDotMax;
    }
    this->thetaDot = this->thetaDDot * (t - this->tInit) - this->thetaDDot * (this->tf - this->tInit);
    this->theta = this->b * (t - this->tf) * (t - this->tf) + this->intermediateThetaRef;
    this->stepComplete = false;
}

/*! This method computes the motor states when a step is complete.
 @return void
 @param t [s] Time the method is called
*/
void StepperMotor::computeStepComplete(double t) {
    this->stepComplete = true;
    this->thetaDDot = 0.0;
    this->thetaDot = 0.0;
    this->theta = this->intermediateThetaRef;

    // Update the motor step count
    if (!this->newMsg) {
        if (this->stepsCommanded > 0) {
            this->stepCount++;
        } else {
            this->stepCount--;
        }
    } else {
        if (this->intermediateThetaRef > this->intermediateThetaInit) {
            this->stepCount++;
        } else {
            this->stepCount--;
        }
    }

    // Update the initial time
    this->tInit = t;

    // Update the actuationComplete boolean variable only when motor actuation is complete
    if ((this->stepCount == this->stepsCommanded) && !this->newMsg) {
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
void StepperMotor::setStepAngle(const double stepAngle) { this->stepAngle = stepAngle; }

/*! Setter method for the motor step time.
 @return void
 @param stepTime [s] Motor step time
*/
void StepperMotor::setStepTime(const double stepTime) { this->stepTime = stepTime; }
