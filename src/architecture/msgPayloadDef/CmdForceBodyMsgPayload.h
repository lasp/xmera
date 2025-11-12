#ifndef _CMD_FORCE_BODY_MESSAGE_
#define _CMD_FORCE_BODY_MESSAGE_

/*! @brief Message used to define the vehicle control force vector in Body frame components*/
typedef struct {
    double forceRequestBody[3];  //!< [N] Control force requested
} CmdForceBodyMsgPayload;

#endif
