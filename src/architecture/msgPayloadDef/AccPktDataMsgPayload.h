#include <stdint.h>

#ifndef _ACC_DATA_MESSAGE_H
#define _ACC_DATA_MESSAGE_H

/*! @brief Structure used to define accelerometer package data */
typedef struct {
    uint64_t measTime;  //!< [Tick] Measurement time for accel
    double gyro_B[3];   //!< [r/s] Angular rate measurement from gyro
    double accel_B[3];  //!< [m/s2] Acceleration in platform frame
} AccPktDataMsgPayload;

#endif
