#ifndef gravityGradientSimMsg_H
#define gravityGradientSimMsg_H

/*! gravity gradient torque message definition */
typedef struct {
    double gravityGradientTorque_B[3];  //!< [Nm] Gravity Gradient torque in body frame components
} GravityGradientMsgPayload;

#endif  // gravityGradientSimMsg_H
