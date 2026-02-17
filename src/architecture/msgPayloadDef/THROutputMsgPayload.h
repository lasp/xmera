// SPDX-License-Identifier: ISC
// Copyright (c) 2016, Autonomous Vehicle System Lab, University of Colorado at Boulder
// Copyright (c) 2025, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

#ifndef SIM_THRUSTER_OUTPUT_MSG_H
#define SIM_THRUSTER_OUTPUT_MSG_H

/*! This structure is used in the messaging system to communicate what the
 state of the vehicle is currently.*/
typedef struct
    //@cond DOXYGEN_IGNORE
    THROutputMsgPayload
//@endcond
{
    double maxThrust;                    //!< N  Steady state thrust of thruster
    double thrustFactor;                 //!< -- Current thrust percentage due to ramp up/down
    double thrustBlowDownFactor;         //!< -- Current thrust percentage due to tank blow down
    double ispBlowDownFactor;            //!< -- Current isp percentage due to tank blow down
    double thrustForce = 0;              //!< N Thrust force magnitude
    double thrustForce_B[3] = {0};       //!< N  Thrust force vector in body frame components
    double thrustTorquePntB_B[3] = {0};  //!< N-m Thrust torque about point B in body frame components
    double thrusterLocation[3] = {0};    //!< m  Current position vector (inertial)
    double thrusterDirection[3] = {0};   //!< -- Unit vector of thruster pointing
} THROutputMsgPayload;

#endif
