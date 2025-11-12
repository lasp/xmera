#ifndef SIM_RW_VOLTAGE_INPUT_H
#define SIM_RW_VOLTAGE_INPUT_H

#include "definitions.h"

/*! @brief Structure used to define the message format of the motor voltage input  */
typedef struct {
    double voltage[RW_EFF_CNT];  //!< [V]     Motor voltage input value
} RwMotorVoltageMsgPayload;

#endif
