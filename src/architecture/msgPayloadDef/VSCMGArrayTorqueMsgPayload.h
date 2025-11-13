#ifndef FSW_VSCMG_TORQUE_H
#define FSW_VSCMG_TORQUE_H

#include "definitions.h"

/*! @brief Structure used to define the output definition for vehicle effectors*/
typedef struct {
    double wheelTorque[RW_EFF_CNT];   //!< [N-m] VSCMG wheel torque array
    double gimbalTorque[RW_EFF_CNT];  //!< [N-m] VSCMG gimbal torque array
} VSCMGArrayTorqueMsgPayload;

#endif
