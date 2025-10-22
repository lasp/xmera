%module miruLowPassFilterConverter
%{
    #include "miruLowPassFilterConverter.h"
%}

%include <std_string.i>
%include <architecture/_GeneralModuleFiles/swig_conly_data.i>

%include <architecture/_GeneralModuleFiles/sys_model.i>
%include "miruLowPassFilterConverter.h"

%include <architecture/msgPayloadDef/AccDataMsgPayload.h>
%include <architecture/msgPayloadDef/IMUSensorMsgPayload.h>
