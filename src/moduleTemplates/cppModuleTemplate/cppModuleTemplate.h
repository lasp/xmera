#ifndef CPP_MODULE_TEMPLATE_H
#define CPP_MODULE_TEMPLATE_H

#include "architecture/_GeneralModuleFiles/sys_model.h"
#include "architecture/messaging/messaging.h"
#include "architecture/msgPayloadDef/ModuleTemplateMsgPayload.h"
#include "architecture/utilities/bskLogging.h"

/*! @brief basic Basilisk C++ module class */
class CppModuleTemplate : public SysModel {
   public:
    CppModuleTemplate();
    ~CppModuleTemplate();

    void reset(uint64_t currentSimNanos);
    void updateState(uint64_t currentSimNanos);

   public:
    double dummy;         //!< [units] sample module variable declaration
    double dumVector[3];  //!< [units] sample vector variable

    Message<ModuleTemplateMsgPayload> dataOutMsg;
    ReadFunctor<ModuleTemplateMsgPayload> dataInMsg;

    BSKLogger bskLogger;  //!< -- BSK Logging
};

#endif
