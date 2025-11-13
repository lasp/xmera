#ifndef XMERA_DEVICESTATUSMSGPAYLOAD_H
#define XMERA_DEVICESTATUSMSGPAYLOAD_H

enum deviceState { On = 1, Off = 0 };

//! @brief Power node command message used to change the state of power modules.
typedef struct {
    enum deviceState deviceStatus;  //!< device status indicator; 0 is off, 1 is on
} DeviceStatusMsgPayload;

#endif  // XMERA_DEVICESTATUSMSGPAYLOAD_H
