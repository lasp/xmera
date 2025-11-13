#ifndef SIM_RW_CMD_H
#define SIM_RW_CMD_H

/*! @brief Structure used to define the individual RW motor torque command message*/
typedef struct {
    double u_cmd;  //!< [Nm], torque command for individual RW
} RWCmdMsgPayload;

#endif
