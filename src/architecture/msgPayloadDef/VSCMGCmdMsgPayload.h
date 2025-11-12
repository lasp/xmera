#ifndef SIM_VSCMG_CMD_H
#define SIM_VSCMG_CMD_H

/*! @brief Structure used to define the individual RW motor torque command message*/
typedef struct {
    double u_s_cmd;  //!< [Nm] torque command for individual wheel
    double u_g_cmd;  //!< [Nm] torque command for individual gimbal
} VSCMGCmdMsgPayload;

#endif
