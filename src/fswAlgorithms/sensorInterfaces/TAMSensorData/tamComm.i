%module tamComm
%{
   #include "tamComm.h"
%}

%include <architecture/_GeneralModuleFiles/sys_model.i>
%include <architecture/_GeneralModuleFiles/swig_conly_data.i>

%include "tamComm.h"

%include <architecture/msgPayloadDef/TAMSensorBodyMsgPayload.h>
%include <architecture/msgPayloadDef/TAMSensorMsgPayload.h>
