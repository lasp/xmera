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

#include "stepperMotorControllerAlgorithm.h"
#include <architecture/utilities/macroDefinitions.h>
#include <stdexcept>

/*! This method performs a complete reset of the module. The input message is checked to ensure it is linked.
 @return void
*/
void StepperMotorControllerAlgorithm::reset() {
    this->stepCount = 0;
    this->stepsCommanded = 0;
    this->previousWrittenTime = -1.0;
}

/*! The update method computes the required number of motor steps given a motor angle reference message. This method
 also tracks the motor actuation in time and includes logic for incoming reference commands that interrupt an unfinished
 motor actuation sequence.
 @return StepperMotorControllerOutput
 @param callTime [ns] Time the method is called
 @param hingedRigidBodyMsgTimeWritten [s] Time the motor reference angle message was written
 @param motorRefAngleIn [-] Motor reference angle message
*/
StepperMotorControllerOutput StepperMotorControllerAlgorithm::update(uint64_t callTime,
                                                                     double hingedRigidBodyMsgTimeWritten,
                                                                     HingedRigidBodyMsgPayload& motorRefAngleIn) {
    StepperMotorControllerOutput stepperMotorControllerOutput{};

    // Each time a new motor reference message is written to this module, the required motor steps commanded to achieve
    // the incoming reference angle are calculated, updated, and output as a MotorStepCommandMsgPayload message
    if (this->previousWrittenTime < hingedRigidBodyMsgTimeWritten) {
        // Update the previous written time
        this->previousWrittenTime = hingedRigidBodyMsgTimeWritten;

        // Set the motor reference angle using the input message
        // (Important: This angle may not be reachable if it is not a multiple of the motor step angle)
        this->thetaRef = motorRefAngleIn.theta;

        // Check that the reference angle is within the actuation region of the motor
        if (this->thetaRef >= this->thetaMax || this->thetaRef <= this->thetaMin) {
            this->thetaRef = this->theta;
            this->stepsCommanded = 0;
        } else {
            // Calculate deltaTheta, the angle the motor must rotate through to achieve the reference angle.
            // Important: The motor cannot stop actuating during a step. If the motor is currently actuating through a
            // step and is interrupted by a new incoming reference message, the motor must complete its actuation
            // through the current step before following the new reference command. Therefore, the motor angle must be
            // rounded up to the nearest multiple of the motor step angle to compute the correct displacement
            // deltaTheta.
            double deltaTheta{};
            if (this->theta > 0) {
                deltaTheta = this->thetaRef - (std::ceil(this->theta / this->stepAngle) * this->stepAngle);
            } else {
                deltaTheta = this->thetaRef - (std::floor(this->theta / this->stepAngle) * this->stepAngle);
            }

            // Calculate the integer number of steps the motor must take to reach the reference angle
            // The exact value is first stored as a double and rounded to the nearest integer step
            double tempStepsCommanded = deltaTheta / this->stepAngle;
            if ((std::ceil(tempStepsCommanded) - tempStepsCommanded) >
                (tempStepsCommanded - std::floor(tempStepsCommanded))) {
                this->stepsCommanded = std::floor(tempStepsCommanded);
            } else {
                this->stepsCommanded = std::ceil(tempStepsCommanded);
            }
        }

        // Use the computed steps commanded to update the motor reference angle to the reachable value
        this->thetaRef = this->theta + (this->stepsCommanded * this->stepAngle);

        // Zero the motor step count because a new reference has been commanded
        this->stepCount = 0;

        MotorStepCommandMsgPayload motorStepCommandOut{};
        motorStepCommandOut.stepsCommanded = this->stepsCommanded;

        stepperMotorControllerOutput.motorStepCommandOut = motorStepCommandOut;
        stepperMotorControllerOutput.writeOutputMessage = true;
    }

    // Calculate the time elapsed since the last motor reference input message was written
    double deltaSimTime = (NANO2SEC * callTime) - this->previousWrittenTime;

    // Update the motor information if steps were commanded
    if (this->stepsCommanded > 0) {
        this->stepCount = std::floor(deltaSimTime / this->stepTime);
        this->theta = this->thetaInit + this->stepAngle * (deltaSimTime / this->stepTime);
        if (this->theta >= this->thetaRef) {
            this->stepCount = this->stepsCommanded;
            this->theta = this->thetaRef;
            this->thetaInit = this->thetaRef;
        }
    } else if (this->stepsCommanded < 0) {
        this->stepCount = -std::floor(deltaSimTime / this->stepTime);
        this->theta = this->thetaInit - this->stepAngle * (deltaSimTime / this->stepTime);
        if (this->theta <= this->thetaRef) {
            this->stepCount = this->stepsCommanded;
            this->theta = this->thetaRef;
            this->thetaInit = this->thetaRef;
        }
    }

    return stepperMotorControllerOutput;
}

/*! Setter method for the initial motor angle.
 @return void
 @param thetaInit [rad] Initial motor angle
*/
void StepperMotorControllerAlgorithm::setThetaInit(const double thetaInit) {
    this->thetaInit = thetaInit;
    this->theta = thetaInit;
}

/*! Getter method for the initial motor angle.
 @return double
*/
double StepperMotorControllerAlgorithm::getThetaInit() const { return this->thetaInit; }

/*! Setter method for the motor upper actuation limit.
 @return void
 @param thetaMax [rad] Motor upper actuation limit
*/
void StepperMotorControllerAlgorithm::setThetaMax(const double thetaMax) { this->thetaMax = thetaMax; }

/*! Getter method for the motor upper actuation limit.
 @return double
*/
double StepperMotorControllerAlgorithm::getThetaMax() const { return this->thetaMax; }

/*! Setter method for the motor lower actuation limit.
 @return void
 @param thetaMin [rad] Motor lower actuation limit
*/
void StepperMotorControllerAlgorithm::setThetaMin(const double thetaMin) { this->thetaMin = thetaMin; }

/*! Getter method for the motor lower actuation limit.
 @return double
*/
double StepperMotorControllerAlgorithm::getThetaMin() const { return this->thetaMin; }

/*! Setter method for the motor step angle.
 @return void
 @param stepAngle [rad] Motor step angle
*/
void StepperMotorControllerAlgorithm::setStepAngle(const double stepAngle) {
    if (stepAngle <= 0.0) {
        throw std::invalid_argument("StepAngle must be positive");
    }
    this->stepAngle = stepAngle;
}

/*! Getter method for the motor step angle.
 @return double
*/
double StepperMotorControllerAlgorithm::getStepAngle() const { return this->stepAngle; }

/*! Setter method for the motor step time.
 @return void
 @param stepTime [s] Motor step time
*/
void StepperMotorControllerAlgorithm::setStepTime(const double stepTime) {
    if (stepTime <= 0.0) {
        throw std::invalid_argument("stepTime must be positive");
    }
    this->stepTime = stepTime;
}

/*! Getter method for the motor step time.
 @return double
*/
double StepperMotorControllerAlgorithm::getStepTime() const { return this->stepTime; }
