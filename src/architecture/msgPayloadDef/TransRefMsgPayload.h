#ifndef TRANS_REF_MESSAGE_H
#define TRANS_REF_MESSAGE_H

/*! @brief Structure used to define the output definition for attitude reference generation */
typedef struct {
    double r_RN_N[3];  //!< [m]     spacecraft reference position vector in inertial frame components
    double v_RN_N[3];  //!< [m/s]  spacecraft reference inertial velocity vector in N frame components
    double a_RN_N[3];  //!< [m/s^2]    spacecraft reference acceleration vector in N frame components
} TransRefMsgPayload;

#endif
