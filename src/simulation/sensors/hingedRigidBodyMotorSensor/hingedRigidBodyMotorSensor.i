%module hingedRigidBodyMotorSensor
%{
    #include "hingedRigidBodyMotorSensor.h"
%}

%include <std_string.i>
%include <architecture/_GeneralModuleFiles/swig_conly_data.i>

%include <architecture/_GeneralModuleFiles/sys_model.i>
%include "hingedRigidBodyMotorSensor.h"

%include <architecture/msgPayloadDef/HingedRigidBodyMsgPayload.h>
