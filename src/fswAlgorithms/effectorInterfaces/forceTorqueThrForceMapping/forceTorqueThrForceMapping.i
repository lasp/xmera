%module forceTorqueThrForceMapping
%{
    #include "forceTorqueThrForceMapping.h"
%}

%include <architecture/_GeneralModuleFiles/sys_model.i>
%include <architecture/_GeneralModuleFiles/swig_conly_data.i>

%include "forceTorqueThrForceMapping.h"
%include "forceTorqueThrForceMappingAlgorithm.h"

%include <architecture/msgPayloadDef/CmdTorqueBodyMsgPayload.h>
%include <architecture/msgPayloadDef/CmdForceBodyMsgPayload.h>
%include <architecture/msgPayloadDef/THRArrayConfigMsgPayload.h>
%include <architecture/msgPayloadDef/VehicleConfigMsgPayload.h>
%include <architecture/msgPayloadDef/THRArrayCmdForceMsgPayload.h>
