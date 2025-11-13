%module simpleInstrument
%{
#include "simpleInstrument.h"
%}

%include <std_string.i>
%include <architecture/_GeneralModuleFiles/sys_model.i>
%include <simulation/onboardDataHandling/_GeneralModuleFiles/dataNodeBase.h>
%include "simpleInstrument.h"
%include <architecture/_GeneralModuleFiles/swig_conly_data.i>

%include <architecture/msgPayloadDef/DataNodeUsageMsgPayload.h>

%include <architecture/msgPayloadDef/DeviceCmdMsgPayload.h>

%include <architecture/msgPayloadDef/DataStorageStatusMsgPayload.h>
