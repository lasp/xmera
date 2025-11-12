#ifndef _TAM_SENSOR_BODY_MESSAGE_H
#define _TAM_SENSOR_BODY_MESSAGE_H

/*! @brief Output structure for TAM measurements*/
typedef struct {
    double tam_B[3];  //!< [Tesla] TAM measurements in vehicle body frame
} TAMSensorBodyMsgPayload;

#endif
