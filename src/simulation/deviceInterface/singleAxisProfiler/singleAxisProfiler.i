%module singleAxisProfiler
%{
   #include "singleAxisProfiler.h"
%}

%include <std_string.i>
%include <architecture/_GeneralModuleFiles/swig_conly_data.i>
%include <architecture/_GeneralModuleFiles/swig_eigen.i>

%include <architecture/_GeneralModuleFiles/sys_model.i>
%include "singleAxisProfiler.h"

%include <architecture/msgPayloadDef/StepperMotorMsgPayload.h>
%include <architecture/msgPayloadDef/PrescribedRotationMsgPayload.h>
