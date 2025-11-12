%module stComm
%{
   #include "stComm.h"
%}

%include <architecture/_GeneralModuleFiles/sys_model.i>
%include <architecture/_GeneralModuleFiles/swig_conly_data.i>

%include "stComm.h"

%include <architecture/msgPayloadDef/STSensorMsgPayload.h>
%include <architecture/msgPayloadDef/STAttMsgPayload.h>
