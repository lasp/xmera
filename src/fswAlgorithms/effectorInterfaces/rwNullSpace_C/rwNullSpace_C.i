%module rwNullSpace_C
%{
   #include "rwNullSpace_C.h"
%}

%include <architecture/_GeneralModuleFiles/sys_model.i>
%include <architecture/_GeneralModuleFiles/swig_conly_data.i>

%include "rwNullSpace_C.h"

%include <architecture/msgPayloadDef/RwMotorTorqueMsgPayload.h>
%include <architecture/msgPayloadDef/RWSpeedMsgPayload.h>
%include <architecture/msgPayloadDef/RWConstellationMsgPayload.h>
