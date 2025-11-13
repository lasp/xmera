%module dvExecuteGuidance
%{
   #include "dvExecuteGuidance.h"
%}

%include <architecture/_GeneralModuleFiles/sys_model.i>
%include <architecture/_GeneralModuleFiles/swig_conly_data.i>

%include "dvExecuteGuidance.h"

%include <architecture/msgPayloadDef/NavTransMsgPayload.h>
%include <architecture/msgPayloadDef/THRArrayOnTimeCmdMsgPayload.h>
%include <architecture/msgPayloadDef/DvBurnCmdMsgPayload.h>
%include <architecture/msgPayloadDef/DvExecutionDataMsgPayload.h>
