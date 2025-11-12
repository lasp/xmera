%module simpleMassProps
%{
   #include "simpleMassProps.h"
%}

%include <std_string.i>
%include <architecture/_GeneralModuleFiles/swig_conly_data.i>

%include <architecture/_GeneralModuleFiles/sys_model.i>
%include "simpleMassProps.h"

%include <architecture/msgPayloadDef/SCMassPropsMsgPayload.h>

%include <architecture/msgPayloadDef/VehicleConfigMsgPayload.h>
