// SPDX-License-Identifier: ISC
// Copyright (c) 2023, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

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
