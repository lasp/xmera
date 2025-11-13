%module sunSearch_C
%{
   #include "sunSearch_C.h"
%}

%include <std_string.i>

%include <architecture/_GeneralModuleFiles/swig_conly_data.i>

%include <architecture/_GeneralModuleFiles/sys_model.i>
%include "sunSearch_C.h"

%include <architecture/msgPayloadDef/NavAttMsgPayload.h>
%include <architecture/msgPayloadDef/VehicleConfigMsgPayload.h>
%include <architecture/msgPayloadDef/AttGuidMsgPayload.h>
