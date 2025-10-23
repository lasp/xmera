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

#ifndef STEPPERMOTORCONTROLLERALGORITHM_H
#define STEPPERMOTORCONTROLLERALGORITHM_H

#include <architecture/msgPayloadDef/HingedRigidBodyMsgPayload.h>
#include <architecture/msgPayloadDef/MotorStepCommandMsgPayload.h>
#include <cmath>
#include <cstdint>

/*! structure containing the stepper motor controller algorithm output */
typedef struct {
    MotorStepCommandMsgPayload motorStepCommandOut; /*!< Output msg for the number of commanded motor steps */
    bool writeOutputMessage;                        /*!< indicator whether or not output message should be written */
} StepperMotorControllerOutput;

/*! @brief Stepper Motor Controller Class */
class StepperMotorControllerAlgorithm {
   public:
    void reset();
    StepperMotorControllerOutput update(uint64_t callTime,
                                        double hingedRigidBodyMsgTimeWritten,
                                        HingedRigidBodyMsgPayload& motorRefAngleIn);
    void setThetaInit(const double thetaInit);
    double getThetaInit() const;
    void setThetaMax(const double thetaMax);
    double getThetaMax() const;
    void setThetaMin(const double thetaMin);
    double getThetaMin() const;
    void setStepAngle(const double stepAngle);
    double getStepAngle() const;
    void setStepTime(const double stepTime);
    double getStepTime() const;

   private:
    double thetaInit{};                    //!< [rad] Initial motor angle
    double theta{};                        //!< [rad] Current motor angle
    double thetaRef{};                     //!< [rad] Motor reference angle
    double stepAngle{1.0 * M_PI / 180.0};  //!< [rad] Step angle the motor rotates through for a single step (constant)
    double thetaMax{2.0 * M_PI};           //!< [rad] Motor upper hard stop actuation limit
    double thetaMin{-2.0 * M_PI};          //!< [rad] Motor lower hard stop actuation limit
    int stepsCommanded{};                  //!< [steps] Number of steps needed to reach the desired angle (output)
    int stepCount{};                       //!< [steps] Current motor step count (number of steps taken)
    double stepTime{1.0};              //!< [s] Time required for the motor to actuate through a single step (constant)
    double previousWrittenTime{-1.0};  //!< [ns] Time the last motor reference input message was written
};

#endif
