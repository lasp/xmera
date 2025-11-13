%module encoder
%{
   #include "encoder.h"
%}

%include <std_string.i>
%include <architecture/_GeneralModuleFiles/swig_conly_data.i>

%include <architecture/_GeneralModuleFiles/sys_model.i>
%include <architecture/utilities/simDefinitions.h>
%include <architecture/utilities/macroDefinitions.h>
%include "encoder.h"

%include <architecture/msgPayloadDef/RWSpeedMsgPayload.h>
