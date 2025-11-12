%module dipoleMapping
%{
    #include "dipoleMapping.h"
%}

%include <architecture/_GeneralModuleFiles/sys_model.i>
%include <architecture/_GeneralModuleFiles/swig_conly_data.i>

%include "dipoleMapping.h"

%include <architecture/msgPayloadDef/DipoleRequestBodyMsgPayload.h>
%include <architecture/msgPayloadDef/MTBArrayConfigMsgPayload.h>
%include <architecture/msgPayloadDef/MTBCmdMsgPayload.h>
