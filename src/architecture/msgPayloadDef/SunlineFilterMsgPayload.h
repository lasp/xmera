#ifndef SUNLINE_FILTER_MESSAGE_H
#define SUNLINE_FILTER_MESSAGE_H

#define SKF_N_STATES 6
#define SKF_N_STATES_SWITCH 6
#define EKF_N_STATES_SWITCH 5
#define SKF_N_STATES_HALF 3
#define MAX_N_CSS_MEAS 32

/*! @brief structure for filter-states output for the unscented kalman filter
 implementation of the sunline state estimator*/
typedef struct {
    double timeTag;                             //!< [s] Current time of validity for output
    double covar[SKF_N_STATES * SKF_N_STATES];  //!< [-] Current covariance of the filter
    double state[SKF_N_STATES];                 //!< [-] Current estimated state of the filter
    double stateError[SKF_N_STATES];            //!< [-] Current deviation of the state from the reference state
    double postFitRes[MAX_N_CSS_MEAS];          //!< [-] PostFit Residuals
    int numObs;                                 //!< [-] Valid observation count for this frame
} SunlineFilterMsgPayload;

#endif
