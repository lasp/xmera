#ifndef REAL_TIME_FACTOR_H
#define REAL_TIME_FACTOR_H

//! @brief Container for sim real time speed up/down factor.
typedef struct {
    double speedFactor;  //!< -- factor of real time at which the sim should run
} RealTimeFactorMsgPayload;

#endif
