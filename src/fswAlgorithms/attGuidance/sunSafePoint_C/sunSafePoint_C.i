%module sunSafePoint_C
%{
   #include "sunSafePoint_C.h"
%}

%include <architecture/_GeneralModuleFiles/sys_model.i>
%include <architecture/_GeneralModuleFiles/swig_conly_data.i>

%include "sunSafePoint_C.h"

%include <architecture/msgPayloadDef/NavAttMsgPayload.h>
%include <architecture/msgPayloadDef/AttGuidMsgPayload.h>
