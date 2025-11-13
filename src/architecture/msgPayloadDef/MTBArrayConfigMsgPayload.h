#ifndef MTB_ARRAY_CONFIG_MSG_H
#define MTB_ARRAY_CONFIG_MSG_H

#include "definitions.h"

/*! @brief magnetic torque bar array configuration msg */
typedef struct {
    int numMTB;                          //!< [-] number of magnetic torque bars on the spacecraft
    double GtMatrix_B[3 * MAX_EFF_CNT];  //!< [-] magnetic torque bar alignment matrix in Body frame components, must be
                                         //!< provided in row-major format
    double maxMtbDipoles[MAX_EFF_CNT];   //!< [A-m2] maximum commandable dipole for each magnetic torque bar
} MTBArrayConfigMsgPayload;

#endif
