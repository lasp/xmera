%module rateControl
%{
   #include "rateControl.h"
%}

%include <std_string.i>
%include <architecture/_GeneralModuleFiles/swig_conly_data.i>
%include <architecture/_GeneralModuleFiles/swig_eigen.i>

%include <architecture/_GeneralModuleFiles/sys_model.i>
%include "rateControl.h"
%include "rateControlAlgorithm.h"

%include <architecture/msgPayloadDef/AttGuidMsgPayload.h>

%include <architecture/msgPayloadDef/VehicleConfigMsgPayload.h>

%include <architecture/msgPayloadDef/CmdTorqueBodyMsgPayload.h>
