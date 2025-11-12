#ifndef _DV_ACCUMULATION_H_
#define _DV_ACCUMULATION_H_

#include <architecture/_GeneralModuleFiles/sys_model.h>
#include <architecture/messaging/messaging.h>
#include <architecture/msgPayloadDef/AccDataMsgPayload.h>
#include <architecture/msgPayloadDef/NavTransMsgPayload.h>
#include <architecture/utilities/bskLogging.h>

/*! @brief Top level structure for the CSS sensor interface system.  Contains all parameters for the
 CSS interface*/
class DVAccumulation : public SysModel {
   public:
    void updateState(uint64_t callTime) override;
    void reset(uint64_t callTime) override;

    Message<NavTransMsgPayload> dvAcumOutMsg;    //!< accumulated DV output message
    ReadFunctor<AccDataMsgPayload> accPktInMsg;  //!< [-] input accelerometer message

    uint32_t msgCount;       //!< [-] The total number of messages read from inputs
    uint32_t dvInitialized;  //!< [-] Flag indicating whether DV has been started completely
    uint64_t previousTime;   //!< [ns] The clock time associated with the previous run of algorithm
    double vehAccumDV_B[3];  //!< [m/s] The accumulated Delta_V in body frame components

    BSKLogger bskLogger{};  //!< BSK Logging
};

void dvAccumulation_QuickSort(AccPktDataMsgPayload* A, int start, int end);

#endif
