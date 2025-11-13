#ifndef AVS_SMALLBODYNAVMSGPAYLOAD_H
#define AVS_SMALLBODYNAVMSGPAYLOAD_H

#define SMALL_BODY_NAV_N_STATES 12

//! @brief Full states and covariances associated with spacecraft navigation about a small body
typedef struct {
    double state[SMALL_BODY_NAV_N_STATES];                           //!< Current state estimate from the filter
    double covar[SMALL_BODY_NAV_N_STATES][SMALL_BODY_NAV_N_STATES];  //!< Current covariance of the filter
} SmallBodyNavMsgPayload;

#endif  // AVS_SMALLBODYNAVMSGPAYLOAD_H
