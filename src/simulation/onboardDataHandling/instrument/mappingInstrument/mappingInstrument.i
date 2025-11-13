%module mappingInstrument
%{
    #include "mappingInstrument.h"
%}

%include <std_string.i>
%include <architecture/_GeneralModuleFiles/swig_conly_data.i>
%include <std_vector.i>
%include <architecture/_GeneralModuleFiles/sys_model.i>
%include "mappingInstrument.h"

%include <architecture/msgPayloadDef/AccessMsgPayload.h>

%include <architecture/msgPayloadDef/DataNodeUsageMsgPayload.h>
