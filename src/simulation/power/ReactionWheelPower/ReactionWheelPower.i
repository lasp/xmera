%module ReactionWheelPower
%{
    #include "ReactionWheelPower.h"
%}

%include <stdint.i>
%include <std_string.i>

%include <architecture/_GeneralModuleFiles/sys_model.i>
%include <simulation/power/_GeneralModuleFiles/powerNodeBase.h>
%include "ReactionWheelPower.h"

%include <architecture/msgPayloadDef/PowerNodeUsageMsgPayload.h>

%include <architecture/msgPayloadDef/DeviceStatusMsgPayload.h>

%include <architecture/msgPayloadDef/RWConfigLogMsgPayload.h>
