#ifndef MTB_MSG_H
#define MTB_MSG_H

/*! gravity gradient torque message definition */
typedef struct {
    double mtbNetTorque_B[3];  //!< [Nm]  net torque contribution of all magnetic torque bars in Body frame components
} MTBMsgPayload;

#endif  // MTB_MSG_H
