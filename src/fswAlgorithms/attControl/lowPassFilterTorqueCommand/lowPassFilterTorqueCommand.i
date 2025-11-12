%module lowPassFilterTorqueCommand
%{
   #include "lowPassFilterTorqueCommand.h"
%}

%include <architecture/_GeneralModuleFiles/sys_model.i>
%include <architecture/_GeneralModuleFiles/swig_conly_data.i>

%include "lowPassFilterTorqueCommand.h"

// sample Module support file to be included in this sub-module
%include <architecture/msgPayloadDef/CmdTorqueBodyMsgPayload.h>
