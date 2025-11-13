%module rasterManager
%{
   #include "rasterManager.h"
%}

%include <architecture/_GeneralModuleFiles/sys_model.i>
%include <architecture/_GeneralModuleFiles/swig_conly_data.i>

%include "rasterManager.h"

%include <architecture/msgPayloadDef/AttStateMsgPayload.h>
