%module simpleVoltEstimator
%{
   #include "simpleVoltEstimator.h"
%}

%include <std_string.i>
%include <architecture/_GeneralModuleFiles/swig_eigen.i>
%include <architecture/_GeneralModuleFiles/swig_conly_data.i>

%include <architecture/_GeneralModuleFiles/sys_model.i>
%include "simpleVoltEstimator.h"

%include <architecture/msgPayloadDef/VoltMsgPayload.h>
