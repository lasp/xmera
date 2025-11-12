#ifndef RW_SPEED_MESSAGE_STRUCT_H
#define RW_SPEED_MESSAGE_STRUCT_H

#include "definitions.h"

/*! @brief Structure used to define the output definition for reaction wheel speeds*/
typedef struct {
    double wheelSpeeds[RW_EFF_CNT];  //!< r/s The current angular velocities of the RW wheel
    double wheelThetas[RW_EFF_CNT];  //!< rad The current angle of the RW if jitter is enabled
} RWSpeedMsgPayload;

#endif
