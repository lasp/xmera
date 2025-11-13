#ifndef _TAM_SENSOR_MESSAGE_H
#define _TAM_SENSOR_MESSAGE_H

//! @brief Simulated TAM Sensor output message definition.
typedef struct {
    double tam_S[3];  //!< [T] Magnetic field measurements in sensor frame
} TAMSensorMsgPayload;

#endif
