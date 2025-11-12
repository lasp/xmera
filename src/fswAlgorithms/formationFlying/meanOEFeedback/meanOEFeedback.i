%module meanOEFeedback
%{
   #include "meanOEFeedback.h"
%}

%include <architecture/_GeneralModuleFiles/sys_model.i>
%include <architecture/_GeneralModuleFiles/swig_conly_data.i>

%include "meanOEFeedback.h"

%include <architecture/msgPayloadDef/NavTransMsgPayload.h>
%include <architecture/msgPayloadDef/CmdForceInertialMsgPayload.h>
