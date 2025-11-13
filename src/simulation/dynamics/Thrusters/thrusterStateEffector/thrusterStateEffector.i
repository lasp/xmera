%module thrusterStateEffector
%{
   #include "thrusterStateEffector.h"
%}

%include <std_string.i>
%include <architecture/_GeneralModuleFiles/swig_eigen.i>
%include <architecture/_GeneralModuleFiles/swig_conly_data.i>

%include <std_vector.i>

%include <architecture/_GeneralModuleFiles/sys_model.i>
%include <simulation/dynamics/_GeneralModuleFiles/stateData.h>
%include <simulation/dynamics/_GeneralModuleFiles/stateEffector.h>
%include <simulation/dynamics/_GeneralModuleFiles/dynParamManager.h>
%include <simulation/dynamics/_GeneralModuleFiles/THRSimConfig.h>
%include "thrusterStateEffector.h"

%include <architecture/msgPayloadDef/THRArrayOnTimeCmdMsgPayload.h>

%include <architecture/msgPayloadDef/THROutputMsgPayload.h>
%include <architecture/msgPayloadDef/SCStatesMsgPayload.h>
