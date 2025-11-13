#ifndef _RW_CONSTELLATION_MESSAGE_H
#define _RW_CONSTELLATION_MESSAGE_H

#include "RWConfigElementMsgPayload.h"
#include "definitions.h"

/*! @brief Message used to define an array of RW FSW configurations  */
typedef struct {
    int numRW;                                             //!< [-] number of RWs
    RWConfigElementMsgPayload reactionWheels[RW_EFF_CNT];  //!< [-] array of the reaction wheels
} RWConstellationMsgPayload;

#endif
