#ifndef ARRAY_MOTOR_FORCE_H
#define ARRAY_MOTOR_FORCE_H

#include "definitions.h"

/*! @brief Structure used to define the output definition for vehicle effectors*/
typedef struct {
    double motorForce[MAX_EFF_CNT];  //!< [N]  motor force array
} ArrayMotorForceMsgPayload;

#endif
