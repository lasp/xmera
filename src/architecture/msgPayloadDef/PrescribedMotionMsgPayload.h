#ifndef prescribedMotionSimMsg_h
#define prescribedMotionSimMsg_h

/*! @brief Structure used to define the individual prescribed motion state effector data message*/
typedef struct {
    double r_FM_M[3];       //!< [m] position vector from the M frame origin to the F frame origin in M frame components
    double rPrime_FM_M[3];  //!< [m/s] B frame time derivative of r_FM_M
    double rPrimePrime_FM_M[3];  //!< [m/s^2] B frame time derivative of rPrime_FM_M
    double omega_FM_F[3];        //!< [rad/s] Angular velocity of the F frame wrt the M frame in F frame components
    double omegaPrime_FM_F[3];   //!< [rad/s^2] B frame time derivative of omega_FM_F
    double sigma_FM[3];          //!< MRP attitude parameters for the F frame relative to the M frame
} PrescribedMotionMsgPayload;

#endif /* prescribedMotionSimMsg_h */
