#ifndef _EPHEMERIS_INTERFACE_DATA_H_
#define _EPHEMERIS_INTERFACE_DATA_H_

/*! @brief time correlation factor structure used to take vehicle time and convert
 it over to ephemeris time (TDB)
 */
typedef struct {
    double ephemerisTime;     //!< [s] Ephemeris time associated with the vehicle time
    double vehicleClockTime;  //!< [s] Vehicle time code converted over to seconds
} TDBVehicleClockCorrelationMsgPayload;

#endif
