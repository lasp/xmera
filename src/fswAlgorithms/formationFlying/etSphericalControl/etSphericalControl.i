%module etSphericalControl
%{
   #include "etSphericalControl.h"
%}

%include <architecture/_GeneralModuleFiles/sys_model.i>
%include <architecture/_GeneralModuleFiles/swig_conly_data.i>

%include "etSphericalControl.h"

%include <architecture/msgPayloadDef/NavTransMsgPayload.h>
%include <architecture/msgPayloadDef/NavAttMsgPayload.h>
%include <architecture/msgPayloadDef/VehicleConfigMsgPayload.h>
%include <architecture/msgPayloadDef/CmdForceInertialMsgPayload.h>
%include <architecture/msgPayloadDef/CmdForceBodyMsgPayload.h>
