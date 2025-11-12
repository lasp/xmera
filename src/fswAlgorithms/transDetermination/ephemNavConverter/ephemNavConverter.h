#ifndef EPHEM_NAV_CONVERTER_H
#define EPHEM_NAV_CONVERTER_H

#include <architecture/_GeneralModuleFiles/sys_model.h>
#include <architecture/messaging/messaging.h>
#include <architecture/msgPayloadDef/EphemerisMsgPayload.h>
#include <architecture/msgPayloadDef/NavTransMsgPayload.h>
#include <architecture/utilities/bskLogging.h>
#include "ephemNavConverterAlgorithm.h"

/*! @brief The ephemNavConverter class.*/
class EphemNavConverter : public SysModel {
   public:
    EphemNavConverter() = default;
    ~EphemNavConverter() = default;

    void reset(uint64_t callTime) override;
    void updateState(uint64_t callTime) override;

    Message<NavTransMsgPayload> stateOutMsg;    //!< [-] output navigation message for pos/vel
    ReadFunctor<EphemerisMsgPayload> ephInMsg;  //!< ephemeris input message

    BSKLogger bskLogger{};  //!< BSK Logging

   private:
    EphemNavConverterAlgorithm algorithm;  //!< Algorithm for ephemNavConverter control logic (BSK-agnostic)
};

#endif
