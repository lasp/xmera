#ifndef _RW_AVAILABILITY_FSW_MSG_H
#define _RW_AVAILABILITY_FSW_MSG_H

#include "definitions.h"
#include "fswAlgorithms/fswUtilities/fswDefinitions.h"

/*! @brief Array with availability of RW */
typedef struct {
    FSWdeviceAvailability wheelAvailability[RW_EFF_CNT];  //!< The current state of the wheel
} RWAvailabilityMsgPayload;

#endif
