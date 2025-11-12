%module starTracker
%{
   #include "starTracker.h"
%}

%include <architecture/_GeneralModuleFiles/swig_conly_data.i>

%include <std_string.i>
%include <architecture/_GeneralModuleFiles/swig_eigen.i>
%include <stdint.i>
%include <architecture/_GeneralModuleFiles/sys_model.i>
%include "starTracker.h"

%include <architecture/msgPayloadDef/SCStatesMsgPayload.h>

%include <architecture/msgPayloadDef/STSensorMsgPayload.h>
