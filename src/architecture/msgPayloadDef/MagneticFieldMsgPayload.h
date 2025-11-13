#ifndef magneticFieldSimMsg_H
#define magneticFieldSimMsg_H

/*! magnetic field message definition */
typedef struct {
    double magField_N[3];  //!< [Tesla] Local magnetic field
} MagneticFieldMsgPayload;
#endif
