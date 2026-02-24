// SPDX-License-Identifier: ISC
// Copyright (c) 2023, Autonomous Vehicle System Lab, University of Colorado at Boulder
// Copyright (c) 2025, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

#ifndef stepperMotorSimMsg_h
#define stepperMotorSimMsg_h

/*! @brief Structure containing stepper motor state information */
typedef struct {
    double theta;        //!< [rad] Current motor angle
    double thetaDot;     //!< [rad/s] Current motor angle rate
    double thetaDDot;    //!< [rad/s^2] Current motor angular acceleration
    int stepsCommanded;  //!< Current number of commanded motor steps
    int stepCount;       //!< Current motor step count (number of steps taken)
} StepperMotorMsgPayload;

#endif /* stepperMotorSimMsg_h */
