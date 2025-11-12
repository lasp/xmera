#ifndef THR_ARRAY_CMD_FORCE_MESSAGE_H_
#define THR_ARRAY_CMD_FORCE_MESSAGE_H_

#include "definitions.h"

/*! @brief Message used to define a vector of thruster force commands */
typedef struct {
    double thrForce[MAX_EFF_CNT];  //!< [N] array of thruster force values
} THRArrayCmdForceMsgPayload;

#endif
