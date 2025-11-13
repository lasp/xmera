%module simplePowerSink
%{
    #include "simplePowerSink.h"
%}

%include <std_string.i>
%include <architecture/_GeneralModuleFiles/swig_eigen.i>

%include <architecture/_GeneralModuleFiles/sys_model.i>
%include <simulation/power/_GeneralModuleFiles/powerNodeBase.h>
%include "simplePowerSink.h"
%include <architecture/_GeneralModuleFiles/swig_conly_data.i>

%include <architecture/msgPayloadDef/PowerNodeUsageMsgPayload.h>

%include <architecture/msgPayloadDef/DeviceStatusMsgPayload.h>
