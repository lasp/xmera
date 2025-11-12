#ifndef SCANNINGINSTRUMENTCONTROLLER_H
#define SCANNINGINSTRUMENTCONTROLLER_H

#include <architecture/_GeneralModuleFiles/sys_model.h>
#include <architecture/messaging/messaging.h>
#include <architecture/msgPayloadDef/AccessMsgPayload.h>
#include <architecture/msgPayloadDef/AttGuidMsgPayload.h>
#include <architecture/msgPayloadDef/DeviceCmdMsgPayload.h>
#include <architecture/msgPayloadDef/DeviceStatusMsgPayload.h>
#include <architecture/utilities/bskLogging.h>
#include <stdint.h>

/*! @brief Module to perform continuous instrument control
 */
class ScanningInstrumentController : public SysModel {
   public:
    void updateState(uint64_t callTime) override;
    void reset(uint64_t callTime) override;

    double attErrTolerance;         //!< Normalized MRP attitude error tolerance
    unsigned int useRateTolerance;  //!< Flag to enable rate error tolerance
    double rateErrTolerance;        //!< Rate error tolerance in rad/s
    unsigned int controllerStatus;  //!< dictates whether or not the controller should be running

    /* declare module IO interfaces */
    ReadFunctor<AccessMsgPayload> accessInMsg;              //!< Ground location access
    ReadFunctor<AttGuidMsgPayload> attGuidInMsg;            //!< Attitude guidance input message
    ReadFunctor<DeviceStatusMsgPayload> deviceStatusInMsg;  //!< Device status input message
    Message<DeviceCmdMsgPayload> deviceCmdOutMsg;           //!< Device status output message

    BSKLogger bskLogger = {};  //!< BSK Logging
};

#endif
