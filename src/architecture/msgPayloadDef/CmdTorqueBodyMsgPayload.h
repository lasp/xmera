#ifndef _CMD_TORQUE_BODY_MESSAGE_
#define _CMD_TORQUE_BODY_MESSAGE_

/*! @brief Message used to define the vehicle control torque vector in Body frame components*/
typedef struct {
    double torqueRequestBody[3];  //!< [Nm] Control torque requested
} CmdTorqueBodyMsgPayload;

#endif
