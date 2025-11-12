#include <stdint.h>

#ifndef DV_BURN_CMD_MESSAGE_H
#define DV_BURN_CMD_MESSAGE_H

/*! @brief Input burn command structure used to configure the burn*/
typedef struct {
    double dvInrtlCmd[3];    //!< [m/s] The commanded DV we need in inertial
    double dvRotVecUnit[3];  //!< [-] The commanded vector we need to rotate about
    double dvRotVecMag;      //!< [r/s] The commanded rotation rate for the vector
    uint64_t burnStartTime;  //!< [ns]  The commanded time to start the burn
} DvBurnCmdMsgPayload;

#endif
