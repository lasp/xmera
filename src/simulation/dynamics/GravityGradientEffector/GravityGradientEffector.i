%module GravityGradientEffector
%{
   #include "GravityGradientEffector.h"
%}

%include <stdint.i>
%include <std_string.i>
%include <architecture/_GeneralModuleFiles/swig_conly_data.i>

// Instantiate templates used by example
%include <architecture/_GeneralModuleFiles/sys_model.i>
%include <simulation/dynamics/_GeneralModuleFiles/stateData.h>
%include <simulation/dynamics/_GeneralModuleFiles/dynamicEffector.h>
%include <simulation/dynamics/_GeneralModuleFiles/dynParamManager.h>

%include "GravityGradientEffector.h"

%include <architecture/msgPayloadDef/GravityGradientMsgPayload.h>
