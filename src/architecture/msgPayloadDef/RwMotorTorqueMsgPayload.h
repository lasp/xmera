#ifndef RW_MOTOR_TORQUE_H
#define RW_MOTOR_TORQUE_H

#include "definitions.h"

/*! @brief Structure used to define the message format of the motor torque */
typedef struct {
    double motorTorque[RW_EFF_CNT];  //!< [Nm]  motor torque array
} RwMotorTorqueMsgPayload;

#endif
