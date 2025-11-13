%module scanningInstrumentController
%{
    #include "scanningInstrumentController.h"
%}

%include <architecture/_GeneralModuleFiles/sys_model.i>
%include <architecture/_GeneralModuleFiles/swig_conly_data.i>

%include "scanningInstrumentController.h"

%include <architecture/msgPayloadDef/AccessMsgPayload.h>
%include <architecture/msgPayloadDef/AttGuidMsgPayload.h>
%include <architecture/msgPayloadDef/DeviceStatusMsgPayload.h>
%include <architecture/msgPayloadDef/DeviceCmdMsgPayload.h>
