#ifndef _CMD_FORCE_INERTIAL_MESSAGE_
#define _CMD_FORCE_INERTIAL_MESSAGE_

/*! @brief Message used to define the vehicle control force vector in Inertial frame components*/
typedef struct {
    double forceRequestInertial[3];  //!< [N] control force request
} CmdForceInertialMsgPayload;

#endif
