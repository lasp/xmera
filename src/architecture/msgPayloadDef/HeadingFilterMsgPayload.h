#ifndef HEADING_FILTER_MESSAGE_H
#define HEADING_FILTER_MESSAGE_H

#define HEAD_N_STATES 3
#define HEAD_N_STATES_SWITCH 5
#define OPNAV_MEAS 3

/*! @brief structure for filter-states output for the unscented kalman filter
 implementation of the sunline state estimator*/
typedef struct {
    double timeTag;                                            /*!< [s] Current time of validity for output */
    double covar[HEAD_N_STATES_SWITCH * HEAD_N_STATES_SWITCH]; /*!< [-] Current covariance of the filter */
    double state[HEAD_N_STATES_SWITCH];                        /*!< [-] Current estimated state of the filter */
    double stateError[HEAD_N_STATES_SWITCH]; /*!< [-] Current deviation of the state from the reference state */
    double postFitRes[3];                    /*!< [-] PostFit Residuals  */

} HeadingFilterMsgPayload;

#endif
