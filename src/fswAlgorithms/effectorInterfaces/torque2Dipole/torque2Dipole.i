%module torque2Dipole
%{
    #include "torque2Dipole.h"
%}

%include <architecture/_GeneralModuleFiles/sys_model.i>
%include <architecture/_GeneralModuleFiles/swig_conly_data.i>

%include "torque2Dipole.h"

%include <architecture/msgPayloadDef/TAMSensorBodyMsgPayload.h>
%include <architecture/msgPayloadDef/DipoleRequestBodyMsgPayload.h>
%include <architecture/msgPayloadDef/CmdTorqueBodyMsgPayload.h>
