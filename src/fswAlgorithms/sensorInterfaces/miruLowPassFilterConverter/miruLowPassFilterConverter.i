%module miruLowPassFilterConverter
%{
    #include "miruLowPassFilterConverter.h"
%}

%include "std_string.i"
%include "swig_conly_data.i"

%include "sys_model.i"
%include "miruLowPassFilterConverter.h"

%include "architecture/msgPayloadDef/AccDataMsgPayload.h"
%include "architecture/msgPayloadDef/IMUSensorMsgPayload.h"
