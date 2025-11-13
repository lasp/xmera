#ifndef ATT_REF_MESSAGE_H
#define ATT_REF_MESSAGE_H

/*! @brief Structure used to define the output definition for attitude reference generation */
typedef struct {
    double sigma_RN[3];     //!<        MRP Reference attitude of R relative to N
    double omega_RN_N[3];   //!< [r/s]  Reference frame rate vector of the of R relative to N in N frame components
    double domega_RN_N[3];  //!< [r/s2] Reference frame inertial acceleration of  R relative to N in N frame components
} AttRefMsgPayload;

#endif
