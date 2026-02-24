// SPDX-License-Identifier: ISC
// Copyright (c) 2016, Autonomous Vehicle System Lab, University of Colorado at Boulder

#ifndef SIM_THRUSTER_PAIR_H
#define SIM_THRUSTER_PAIR_H

//! @brief Container for time/value pairs for ramp on and ramp off profiles.
/*! This structure is used to build up the thruster on and thruster off ramps.
 It pairs the time-delta from on command and thrust factor (percentage/100.0)
 for the entire ramp.*/
typedef struct {
    double ThrustFactor;  //!< --  Percentage of max thrust
    double IspFactor;     //!< [s] fraction specific impulse
    double TimeDelta;     //!< [s] Time delta from start of event
} THRTimePair;

#endif
