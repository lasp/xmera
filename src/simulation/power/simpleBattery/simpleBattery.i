%module simpleBattery
%{
    #include "simpleBattery.h"
%}

%include <std_string.i>
%include <swig_eigen.i>

%include <architecture/_GeneralModuleFiles/sys_model.i>
%include <simulation/power/_GeneralModuleFiles/powerStorageBase.h>
%include "simpleBattery.h"
%include <architecture/_GeneralModuleFiles/swig_conly_data.i>

%include <architecture/msgPayloadDef/PowerNodeUsageMsgPayload.h>

%include <architecture/msgPayloadDef/PowerStorageStatusMsgPayload.h>
