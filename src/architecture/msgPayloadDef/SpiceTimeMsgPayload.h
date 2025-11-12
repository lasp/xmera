#ifndef SPICE_TIME_MESSAGE_H
#define SPICE_TIME_MESSAGE_H

//! The SPICE time output structure outputs time information to the rest of the system
typedef struct {
    double J2000Current;       //!< s Current J2000 elapsed time
    double JulianDateCurrent;  //!< s Current JulianDate
    double GPSSeconds;         //!< s Current GPS seconds
    uint16_t GPSWeek;          //!< -- Current GPS week value
    uint64_t GPSRollovers;     //!< -- Count on the number of GPS rollovers
} SpiceTimeMsgPayload;

#endif
