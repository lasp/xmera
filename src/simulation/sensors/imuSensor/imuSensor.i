%module imuSensor
%{
   #include "imuSensor.h"
%}

%include <architecture/_GeneralModuleFiles/swig_conly_data.i>

%include <std_string.i>
%include <architecture/_GeneralModuleFiles/swig_eigen.i>
%include <architecture/_GeneralModuleFiles/sys_model.i>
%include "imuSensor.h"

%include <architecture/msgPayloadDef/SCStatesMsgPayload.h>

%include <architecture/msgPayloadDef/IMUSensorMsgPayload.h>
