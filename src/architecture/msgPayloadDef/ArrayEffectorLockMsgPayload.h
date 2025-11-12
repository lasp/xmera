#ifndef ARRAY_EFFECTOR_LOCK_H
#define ARRAY_EFFECTOR_LOCK_H

#include "definitions.h"

/*! @brief Structure used to define the output definition for vehicle effectors*/
typedef struct {
    int effectorLockFlag[MAX_EFF_CNT];  //!< effector lock flag; 0 : do not lock; 1 : lock
} ArrayEffectorLockMsgPayload;

#endif
