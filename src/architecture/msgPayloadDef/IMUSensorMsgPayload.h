#ifndef _IMU_SENSOR_MESSAG_H
#define _IMU_SENSOR_MESSAG_H

//! @brief Simulated IMU Sensor output message definition.
typedef struct {
    double DVFramePlatform[3];             //!< m/s Accumulated DVs in platform
    double AccelPlatform[3];               //!< m/s2 Apparent acceleration of the platform
    double DRFramePlatform[3];             //!< r  Accumulated DRs in platform
    double AngVelPlatform[3];              //!< r/s Angular velocity in platform frame
    double timeTag;                        //!< [-] Observation time tag
    double numberOfValidGyroMeasurements;  //!< Number of valid gyro measurements
} IMUSensorMsgPayload;

#endif
