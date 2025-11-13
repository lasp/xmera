#ifndef THR_CMD_MESSAGE_H
#define THR_CMD_MESSAGE_H

#include "definitions.h"

/*! @brief Structure used to define the output definition for vehicle effectors*/
typedef struct {
    double OnTimeRequest[MAX_EFF_CNT];  //!< - Control request fraction array
} THRArrayOnTimeCmdMsgPayload;

#endif
