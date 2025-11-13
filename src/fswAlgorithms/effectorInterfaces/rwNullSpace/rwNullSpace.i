%module rwNullSpace
%{
   #include "rwNullSpace.h"
%}

%include <architecture/_GeneralModuleFiles/sys_model.i>
%include <architecture/_GeneralModuleFiles/swig_conly_data.i>

%include "rwNullSpace.h"
%include "rwNullSpaceAlgorithm.h"

%include <architecture/msgPayloadDef/RwMotorTorqueMsgPayload.h>
%include <architecture/msgPayloadDef/RWSpeedMsgPayload.h>
%include <architecture/msgPayloadDef/RWConstellationMsgPayload.h>
