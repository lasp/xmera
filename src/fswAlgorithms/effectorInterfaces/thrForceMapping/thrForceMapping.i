%module thrForceMapping
%{
    #include "thrForceMapping.h"
%}

%include <architecture/_GeneralModuleFiles/sys_model.i>
%include <architecture/_GeneralModuleFiles/swig_common_model.i>
%include <architecture/_GeneralModuleFiles/swig_conly_data.i>
EIGEN_MAT_WRAP(Vector36d, 157)
%include "thrForceMapping.h"
%include "thrForceMappingAlgorithm.h"

%include <architecture/msgPayloadDef/THRArrayCmdForceMsgPayload.h>
%include <architecture/msgPayloadDef/VehicleConfigMsgPayload.h>
%include <architecture/msgPayloadDef/THRArrayConfigMsgPayload.h>
%include <architecture/msgPayloadDef/CmdTorqueBodyMsgPayload.h>
