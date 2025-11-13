%module simpleDeadband
%{
   #include "simpleDeadband.h"
%}

%include <architecture/_GeneralModuleFiles/sys_model.i>
%include <architecture/_GeneralModuleFiles/swig_conly_data.i>

%include "simpleDeadband.h"

%include <architecture/msgPayloadDef/AttGuidMsgPayload.h>
