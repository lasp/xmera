#ifndef TEMPERATURE_H
#define TEMPERATURE_H

/*! @brief Message for reporting the power consumed produced or consumed by a module.*/
typedef struct {
    double temperature;  //!< [Celsius] current temperature
} TemperatureMsgPayload;

#endif  // TEMPERATURE_H
