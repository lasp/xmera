#ifndef VSCMG_SPEED_MESSAGE_STRUCT_H
#define VSCMG_SPEED_MESSAGE_STRUCT_H

#include "definitions.h"

/*! @brief Structure used to define the output definition for VSCMG speeds*/
typedef struct {
    double wheelSpeeds[RW_EFF_CNT];   //!< r/s The current angular velocities of the VSCMG wheel
    double gimbalAngles[RW_EFF_CNT];  //!< r The current angles of the VSCMG gimbal
    double gimbalRates[RW_EFF_CNT];   //!< r/s The current angular velocities of the VSCMG gimbal
} VSCMGSpeedMsgPayload;

#endif
