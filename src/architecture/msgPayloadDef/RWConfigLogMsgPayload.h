// SPDX-License-Identifier: ISC
// Copyright (c) 2016, Autonomous Vehicle System Lab, University of Colorado at Boulder
// Copyright (c) 2025, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

#ifndef SIM_RW_CONFIG_LOG_MESSAGE_H
#define SIM_RW_CONFIG_LOG_MESSAGE_H

#include "simulation/dynamics/reactionWheels/reactionWheelSupport.h"

/*! @brief Structure used to define the individual RW configuration data message*/
typedef struct {
    double rWB_B[3];             //!< [m], position vector of the RW relative to the spacecraft body frame
    double gsHat_B[3];           //!< [-] spin axis unit vector in body frame
    double w2Hat0_B[3];          //!< [-] initial torque axis unit vector in body frame
    double w3Hat0_B[3];          //!< [-] initial gimbal axis unit vector in body frame
    double mass;                 //!< [kg], reaction wheel rotor mass
    double theta;                //!< [rad], wheel angle
    double Omega;                //!< [rad/s], wheel speed
    double Js;                   //!< [kg-m^2], spin axis gsHat rotor moment of inertia
    double Jt;                   //!< [kg-m^2], gtHat axis rotor moment of inertia
    double Jg;                   //!< [kg-m^2], ggHat axis rotor moment of inertia
    double U_s;                  //!< [kg-m], static imbalance
    double U_d;                  //!< [kg-m^2], dynamic imbalance
    double d;                    //!< [m], wheel center of mass offset from wheel frame origin
    double J13;                  //!< [kg-m^2], x-z inertia of wheel about wheel center in wheel frame (imbalance)
    double u_current;            //!< [N-m], current motor torque
    double frictionTorque;       //!< [N-m], friction Torque
    double u_max;                //!< [N-m], Max torque
    double u_min;                //!< [N-m], Min torque
    double u_f;                  //!< [N-m], Coulomb friction torque magnitude
    double Omega_max;            //!< [rad/s], max wheel speed
    double P_max;                //!< [N-m/s], maximum wheel power
    double linearFrictionRatio;  //!< [%] ratio relative to max speed value up to which the friction behaves linearly
    RWModels RWModel;            //!< [-], Type of imbalance model to use
} RWConfigLogMsgPayload;

#endif
