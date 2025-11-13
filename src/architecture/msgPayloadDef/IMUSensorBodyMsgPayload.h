#ifndef _IMU_SENSOR_BODY_MESSAGE_H
#define _IMU_SENSOR_BODY_MESSAGE_H

/*! @brief Output structure for IMU structure in vehicle body frame*/
typedef struct {
    double DVFrameBody[3];  //!< m/s Accumulated DVs in body
    double AccelBody[3];    //!< m/s2 Apparent acceleration of the body
    double DRFrameBody[3];  //!< r  Accumulated DRs in body
    double AngVelBody[3];   //!< r/s Angular velocity in platform body
} IMUSensorBodyMsgPayload;

#endif
