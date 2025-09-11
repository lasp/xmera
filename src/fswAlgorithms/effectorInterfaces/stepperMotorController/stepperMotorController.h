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

#ifndef _STEPPERMOTORCONTROLLER_
#define _STEPPERMOTORCONTROLLER_

#include "architecture/_GeneralModuleFiles/sys_model.h"
#include "architecture/messaging/messaging.h"
#include "architecture/msgPayloadDef/HingedRigidBodyMsgPayload.h"
#include "architecture/msgPayloadDef/MotorStepCommandMsgPayload.h"
#include <cmath>
#include <cstdint>

/*! @brief Stepper Motor Controller Class */
class StepperMotorController : public SysModel {
   public:
    void reset(uint64_t currentSimNanos) override;        //!< Reset member function
    void updateState(uint64_t currentSimNanos) override;  //!< Update member function
    void setThetaInit(const double thetaInit);            //!< Setter method for the initial motor angle
    double getThetaInit() const;                          //!< Getter method for the initial motor angle
    void setThetaMax(const double thetaMax);              //!< Setter method for the motor upper actuation limit
    double getThetaMax() const;                           //!< Getter method for the motor upper actuation limit
    void setThetaMin(const double thetaMin);              //!< Setter method for the motor lower actuation limit
    double getThetaMin() const;                           //!< Getter method for the motor lower actuation limit
    void setStepAngle(const double stepAngle);            //!< Setter method for the motor step angle
    double getStepAngle() const;                          //!< Getter method for the motor step angle
    void setStepTime(const double stepTime);              //!< Setter method for the motor step time
    double getStepTime() const;                           //!< Getter method for the motor step time

    ReadFunctor<HingedRigidBodyMsgPayload> motorRefAngleInMsg;   //!< Intput msg for the motor reference angle message
    Message<MotorStepCommandMsgPayload> motorStepCommandOutMsg;  //!< Output msg for the number of commanded motor steps

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
