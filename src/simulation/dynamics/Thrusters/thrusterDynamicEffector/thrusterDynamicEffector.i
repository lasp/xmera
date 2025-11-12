%module thrusterDynamicEffector
%{
   #include "thrusterDynamicEffector.h"
%}

%include "std_string.i"
%include "swig_eigen.i"
%include "swig_conly_data.i"

// Instantiate templates used by example
%include "std_vector.i"
namespace std {
    %template(ThrusterTimeVector) vector<THRTimePair, std::allocator<THRTimePair>>;
    %template(ThrusterConfigVector) vector<THRSimConfig, std::allocator<THRSimConfig>>;
}

%include "sys_model.i"
%include "simulation/dynamics/_GeneralModuleFiles/stateData.h"
%include "simulation/dynamics/_GeneralModuleFiles/dynamicEffector.h"
%include "simulation/dynamics/_GeneralModuleFiles/dynParamManager.h"
%include "thrusterDynamicEffector.h"

%include "simulation/dynamics/_GeneralModuleFiles/THRTimePair.h"
%include "simulation/dynamics/_GeneralModuleFiles/THRSimConfig.h"

%include "architecture/msgPayloadDef/THRArrayOnTimeCmdMsgPayload.h"

%include "architecture/msgPayloadDef/THROutputMsgPayload.h"
%include "architecture/msgPayloadDef/SCStatesMsgPayload.h"
