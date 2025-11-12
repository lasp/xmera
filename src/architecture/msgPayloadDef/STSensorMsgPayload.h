#ifndef _ST_HW_OUTPUT_
#define _ST_HW_OUTPUT_
#include <stdint.h>

/*! @brief Star tracker sensor message */
typedef struct {
    double timeTag;         //!< [s] Time tag placed on the output state
    double qInrtl2Case[4];  //!< [-] Quaternion to go from the inertial to case
    double omega_CN_C[3];   //!< [rad/s] Inertial angular velocity in case frame
} STSensorMsgPayload;

#endif
