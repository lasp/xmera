#ifndef RW_CONFIG_MESSAGE_H
#define RW_CONFIG_MESSAGE_H

#include "definitions.h"
#include <stdint.h>

/*! @brief RW array configuration FSW msg */
typedef struct {
    double GsMatrix_B[3 * RW_EFF_CNT];  //!< [-]    The RW spin axis matrix in body frame components
    double JsList[RW_EFF_CNT];          //!< [kgm2] The spin axis inertia for RWs
    int numRW;                          //!< [-]    The number of reaction wheels available on vehicle
    double uMax[RW_EFF_CNT];            //!< [Nm]   The maximum RW motor torque
} RWArrayConfigMsgPayload;

#endif
