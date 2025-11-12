#ifndef prescribedTranslationSimMsg_h
#define prescribedTranslationSimMsg_h

/*! @brief Structure used to define the prescribed motion state effector translational state data message */
typedef struct {
    double r_FM_M[3];       //!< [m] Position vector from the M frame origin to the F frame origin expressed in M frame
                            //!< components
    double rPrime_FM_M[3];  //!< [m/s] B/M frame time derivative of r_FM_M
    double rPrimePrime_FM_M[3];  //!< [m/s^2] B/M frame time derivative of rPrime_FM_M
} PrescribedTranslationMsgPayload;

#endif /* prescribedTranslationSimMsg_h */
