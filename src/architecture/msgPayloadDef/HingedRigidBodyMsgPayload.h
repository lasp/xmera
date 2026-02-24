// SPDX-License-Identifier: ISC
// Copyright (c) 2016, Autonomous Vehicle System Lab, University of Colorado at Boulder
// Copyright (c) 2025, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

#ifndef hingedRigidBodySimMsg_h
#define hingedRigidBodySimMsg_h

/*! @brief Structure used to define the individual Hinged Rigid Body  data message*/
typedef struct {
    double theta;     //!< [rad], panel angular displacement
    double thetaDot;  //!< [rad/s], panel angular displacement rate
} HingedRigidBodyMsgPayload;

#endif /* hingedRigidBodySimMsg_h */
