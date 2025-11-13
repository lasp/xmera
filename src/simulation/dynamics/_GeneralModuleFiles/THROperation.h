#ifndef SIM_THRUSTER_OPERATION_H
#define SIM_THRUSTER_OPERATION_H
#include <stdint.h>

//! @brief Container for current operational data of a given thruster
/*! This structure is used to determine the current state of a given thruster.
 It defines where in the cycle the thruster is and how much longer it should be
 on for.  It is intended to have the previous firing remain resident for logging*/
typedef struct
    //@cond DOXYGEN_IGNORE
    THROperation
//@endcond
{
    uint64_t fireCounter;                  //!< (-) Number of times thruster fired
    double ThrustFactor;                   //!< -- Current thrust scaling factor due to ramp up/down
    double IspFactor;                      //!< -- Current fractional ISP
    double thrustBlowDownFactor = 1.0;     //!< -- Current thrust scaling factor due to tank blow down
    double ispBlowDownFactor = 1.0;        //!< -- Current Isp scaling due to tank blow down
    double ThrustOnCmd;                    //!< s  Time Thruster was requested
    double totalOnTime;                    //!< s  Total amount of time thruster has fired
    double opThrustForce_B[3] = {0};       //!< N  Thrust force vector in body frame components
    double opThrustTorquePntB_B[3] = {0};  //!< N-m Thrust torque about point B in body frame components

    // The following are used in thrusterDynamicEffector
    double ThrustOnRampTime;    //!< s  Time thruster has been on for
    double ThrustOnSteadyTime;  //!< s  Time thruster has been on steady
    double ThrustOffRampTime;   //!< s  Time thruster has been turning off
    double ThrusterStartTime;   //!< s  Time thruster has been executing total
    double PreviousIterTime;    //!< s  Previous thruster int time

    // The following is used in thrusterStateEffector
    double ThrusterEndTime;  //!< s  Time thruster stops burning
} THROperation;

#endif
