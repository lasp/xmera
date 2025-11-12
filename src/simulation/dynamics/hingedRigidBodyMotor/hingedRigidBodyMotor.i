%module hingedRigidBodyMotor
%{
    #include "hingedRigidBodyMotor.h"
%}

%include <std_string.i>
%include <architecture/_GeneralModuleFiles/swig_conly_data.i>

%include <architecture/_GeneralModuleFiles/sys_model.i>
%include "hingedRigidBodyMotor.h"

%include <architecture/msgPayloadDef/HingedRigidBodyMsgPayload.h>

%include <architecture/msgPayloadDef/ArrayMotorTorqueMsgPayload.h>
