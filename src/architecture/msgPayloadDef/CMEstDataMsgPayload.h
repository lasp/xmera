#ifndef _CM_EST_DATA_MESSAGE_H
#define _CM_EST_DATA_MESSAGE_H

/*! @brief Structure used to define CM estimation output data */
typedef struct {
    double attError;       //!< [-]   attitude convergence error
    double state[3];       //!< [m]   estimated cm location
    double stateError[3];  //!< [m]   errpr w.r.t. truth
    double covariance[3];  //!< [m^2] CM estimated covariance
    double preFitRes[3];   //!< [Nm]  pre-fit torque residuals
    double postFitRes[3];  //!< [Nm]  post-fit torque residuals
} CMEstDataMsgPayload;

#endif
