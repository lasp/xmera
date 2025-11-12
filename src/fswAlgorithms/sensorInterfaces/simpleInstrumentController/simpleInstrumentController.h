#ifndef _SIMPLE_INSTRUMENT_CONTROLLER_H_
#define _SIMPLE_INSTRUMENT_CONTROLLER_H_

#include <architecture/_GeneralModuleFiles/sys_model.h>
#include <architecture/messaging/messaging.h>
#include <architecture/msgPayloadDef/AccessMsgPayload.h>
#include <architecture/msgPayloadDef/AttGuidMsgPayload.h>
#include <architecture/msgPayloadDef/DeviceCmdMsgPayload.h>
#include <architecture/msgPayloadDef/DeviceStatusMsgPayload.h>
#include <architecture/utilities/bskLogging.h>
#include <stdint.h>

/*! @brief Data configuration structure for the MRP feedback attitude control routine. */
class SimpleInstrumentController : public SysModel {
   public:
    void updateState(uint64_t callTime) override;
    void reset(uint64_t callTime) override;

    /* User configurable variables */
    double attErrTolerance;         //!< Normalized MRP attitude error tolerance
    unsigned int useRateTolerance;  //!< Flag to enable rate error tolerance
    double rateErrTolerance;        //!< Rate error tolerance in rad/s
    unsigned int imaged;            //!< Indicator for whether or not the image has already been captured
    unsigned int controllerStatus;  //!< dictates whether or not the controller should be running

    /* declare module IO interfaces */
    ReadFunctor<AccessMsgPayload> locationAccessInMsg;      //!< Ground location access input message
    ReadFunctor<AttGuidMsgPayload> attGuidInMsg;            //!< attitude guidance input message
    ReadFunctor<DeviceStatusMsgPayload> deviceStatusInMsg;  //!< (optional) device status input message
    Message<DeviceCmdMsgPayload> deviceCmdOutMsg;           //!< device status output message

    BSKLogger bskLogger = {};  //!< BSK Logging
};

#endif  //_SIMPLE_INSTRUMENT_CONTROLLER_H_
