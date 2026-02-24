// SPDX-License-Identifier: ISC
// Copyright (c) 2023, Autonomous Vehicle System Lab, University of Colorado at Boulder
// Copyright (c) 2025, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

#ifndef motorStepCommandSimMsg_h
#define motorStepCommandSimMsg_h

/*! @brief Structure containing number of commanded stepper motor steps */
typedef struct {
    int stepsCommanded;  //!< Number of commanded stepper motor steps
} MotorStepCommandMsgPayload;

#endif /* motorStepCommandSimMsg_h */
