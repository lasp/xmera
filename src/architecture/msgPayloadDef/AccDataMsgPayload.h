#ifndef _ACC_PKT_DATA_MESSAGE_H
#define _ACC_PKT_DATA_MESSAGE_H

#define MAX_ACC_BUF_PKT 120

#include "AccPktDataMsgPayload.h"

/*! @brief Structure used to define accelerometer package data */
typedef struct {
    AccPktDataMsgPayload accPkts[MAX_ACC_BUF_PKT];  //!< [-] Accelerometer buffer read in
} AccDataMsgPayload;

#endif
