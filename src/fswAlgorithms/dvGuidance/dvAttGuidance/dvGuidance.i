%module dvGuidance
%{
   #include "dvGuidance.h"
%}

%include <architecture/_GeneralModuleFiles/sys_model.i>
%include <architecture/_GeneralModuleFiles/swig_conly_data.i>

%include "dvGuidance.h"

%include <architecture/msgPayloadDef/AttRefMsgPayload.h>
%include <architecture/msgPayloadDef/DvBurnCmdMsgPayload.h>
