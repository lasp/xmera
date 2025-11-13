#ifndef XMERA_GROUNDPOSITIONSIMMSG_H
#define XMERA_GROUNDPOSITIONSIMMSG_H

/*! @brief Message that defines the inertial location of a groundLocation at the current time.
 */
typedef struct {
    double r_LN_N[3];  //!< Position vector of the location w.r.t. the inertial origin in the inertial frame
    double r_LP_N[3];  //!< Position vector of the location with respect to the planet center in the inertial frame
} GroundStateMsgPayload;

#endif  // XMERA_GROUNDPOSITIONSIMMSG_H
