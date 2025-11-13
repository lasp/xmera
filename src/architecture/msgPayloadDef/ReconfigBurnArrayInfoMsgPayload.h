#ifndef RECONFIG_BURN_ARRAY_INFO_H
#define RECONFIG_BURN_ARRAY_INFO_H

#define MAX_BURN_CNT 3

#include "ReconfigBurnInfoMsgPayload.h"

//! @brief Container for the orbit reconfiguration burn information.
typedef struct {
    ReconfigBurnInfoMsgPayload burnArray[MAX_BURN_CNT];  //!< array of burn info messages
} ReconfigBurnArrayInfoMsgPayload;

#endif
