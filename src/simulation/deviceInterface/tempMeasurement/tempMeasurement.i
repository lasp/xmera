%module tempMeasurement
%{
    #include "tempMeasurement.h"
%}

%include <std_string.i>
%include <architecture/_GeneralModuleFiles/swig_conly_data.i>

%include <architecture/_GeneralModuleFiles/sys_model.i>
%include "tempMeasurement.h"

%include <architecture/msgPayloadDef/TemperatureMsgPayload.h>
