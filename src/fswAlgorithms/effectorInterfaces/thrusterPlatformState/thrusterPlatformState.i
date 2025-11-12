%module thrusterPlatformState
%{
   #include "thrusterPlatformState.h"
%}

%include <architecture/_GeneralModuleFiles/sys_model.i>
%include <architecture/_GeneralModuleFiles/swig_conly_data.i>

%include "thrusterPlatformState.h"

%include <architecture/msgPayloadDef/THRConfigMsgPayload.h>
%include <architecture/msgPayloadDef/HingedRigidBodyMsgPayload.h>
