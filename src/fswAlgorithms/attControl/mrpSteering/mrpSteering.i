%module mrpSteering
%{
   #include "mrpSteering.h"
%}

%include <architecture/_GeneralModuleFiles/sys_model.i>
%include <architecture/_GeneralModuleFiles/swig_conly_data.i>

%include "mrpSteering.h"
%include "mrpSteeringAlgorithm.h"

%include <architecture/msgPayloadDef/AttGuidMsgPayload.h>
%include <architecture/msgPayloadDef/RateCmdMsgPayload.h>
