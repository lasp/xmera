#ifndef ARRAY_MOTOR_TORQUE_H
#define ARRAY_MOTOR_TORQUE_H

#include "definitions.h"

/*! @brief Structure used to define the output definition for vehicle effectors*/
typedef struct {
    double motorTorque[MAX_EFF_CNT];  //!< [Nm]  motor torque array
} ArrayMotorTorqueMsgPayload;

#endif
