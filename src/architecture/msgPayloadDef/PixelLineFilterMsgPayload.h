#ifndef PIXLINE_FILTER_MESSAGE_H
#define PIXLINE_FILTER_MESSAGE_H

#define PIXLINE_N_STATES 9
#define PIXLINE_DYN_STATES 6
#define PIXLINE_N_MEAS 6

/*! @brief structure for filter-states output for the unscented kalman filter
 implementation of the sunline state estimator*/
typedef struct {
    double timeTag;                                     //!< [s] Current time of validity for output
    double covar[PIXLINE_N_STATES * PIXLINE_N_STATES];  //!< [-] Current covariance of the filter
    double state[PIXLINE_N_STATES];                     //!< [-] Current estimated state of the filter
    double stateError[PIXLINE_N_STATES];                //!< [-] Current deviation of the state from the reference state
    double postFitRes[PIXLINE_N_MEAS];                  //!< [-] PostFit Residuals
    int numObs;                                         //!< [-] Valid observation count for this frame
} PixelLineFilterMsgPayload;

#endif
