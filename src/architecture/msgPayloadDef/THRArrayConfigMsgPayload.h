#ifndef THR_ARRAY_MESSAGE_H
#define THR_ARRAY_MESSAGE_H

#include "THRConfigMsgPayload.h"
#include "definitions.h"

/*! @brief FSW message definition containing the thruster cluster information */
typedef struct {
    uint32_t numThrusters;                       //!< [-] number of thrusters
    THRConfigMsgPayload thrusters[MAX_EFF_CNT];  //!< [-] array of thruster configuration information
} THRArrayConfigMsgPayload;

#endif
