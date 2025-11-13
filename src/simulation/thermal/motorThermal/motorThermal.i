%module motorThermal
%{
   #include "motorThermal.h"
%}

%include <std_string.i>
%include <architecture/_GeneralModuleFiles/swig_conly_data.i>

%include <architecture/_GeneralModuleFiles/sys_model.i>
%include "motorThermal.h"

%include <architecture/msgPayloadDef/TemperatureMsgPayload.h>


%include <architecture/msgPayloadDef/RWConfigLogMsgPayload.h>
