#ifndef MTB_CMD_MSG_H
#define MTB_CMD_MSG_H

#include "definitions.h"

/*! @brief Message for magnetic torque bar dipole commands. */
typedef struct {
    double mtbDipoleCmds[MAX_EFF_CNT];  //!< [A-m2] magnetic torque bar dipole cmds
} MTBCmdMsgPayload;

#endif
