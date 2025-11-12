%module imuComm
%{
   #include "imuComm.h"
%}

%include <architecture/_GeneralModuleFiles/sys_model.i>
%include <architecture/_GeneralModuleFiles/swig_conly_data.i>

%include "imuComm.h"

%include <architecture/msgPayloadDef/IMUSensorBodyMsgPayload.h>
%include <architecture/msgPayloadDef/IMUSensorMsgPayload.h>
