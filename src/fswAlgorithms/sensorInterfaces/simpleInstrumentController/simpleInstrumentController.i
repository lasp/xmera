%module simpleInstrumentController
%{
#include "simpleInstrumentController.h"
%}

%include <architecture/_GeneralModuleFiles/sys_model.i>
%include <architecture/_GeneralModuleFiles/swig_conly_data.i>

%include "simpleInstrumentController.h"

%include <architecture/msgPayloadDef/AttGuidMsgPayload.h>
%include <architecture/msgPayloadDef/AccessMsgPayload.h>
%include <architecture/msgPayloadDef/DeviceCmdMsgPayload.h>
%include <architecture/msgPayloadDef/DeviceStatusMsgPayload.h>
