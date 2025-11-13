%module vehicleConfigData
%{
   #include "vehicleConfigData.h"
%}

%include <architecture/_GeneralModuleFiles/sys_model.i>
%include <architecture/_GeneralModuleFiles/swig_conly_data.i>
%include "vehicleConfigData.h"

%include <architecture/msgPayloadDef/VehicleConfigMsgPayload.h>
