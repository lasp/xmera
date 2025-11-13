%module extForceTorque
%{
   #include "extForceTorque.h"
%}

%include <std_string.i>
%include <architecture/_GeneralModuleFiles/swig_eigen.i>
%include <architecture/_GeneralModuleFiles/swig_conly_data.i>

%include <architecture/_GeneralModuleFiles/sys_model.i>
%include <simulation/dynamics/_GeneralModuleFiles/dynamicEffector.h>

%include "extForceTorque.h"

%include <architecture/msgPayloadDef/CmdTorqueBodyMsgPayload.h>

%include <architecture/msgPayloadDef/CmdForceBodyMsgPayload.h>

%include <architecture/msgPayloadDef/CmdForceInertialMsgPayload.h>
