#ifndef XMERA_DEVICECMDMSGPAYLOAD_H
#define XMERA_DEVICECMDMSGPAYLOAD_H

#include <cstdint>

//! @brief Device command message used to change the state of instruments.
typedef struct {
    uint64_t deviceCmd;  //!< device command; 0 is off, >0 commands other states
} DeviceCmdMsgPayload;

#endif  // XMERA_DEVICECMDMSGPAYLOAD_H
